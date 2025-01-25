using System.Reactive.Concurrency;
using System.Text.Json;
using Google.Protobuf;
using Meshtastic.Connections;
using Meshtastic.Data.MessageFactories;
using Meshtastic.Extensions;
using Meshtastic.Protobufs;
using Microsoft.EntityFrameworkCore;
using Monitor.Context;
using Monitor.Context.Entities;
using Monitor.Extensions;
using Monitor.Models;
using Monitor.Services;
using Exception = System.Exception;

namespace Monitor.Workers.Enableables;

public class MeshtasticApp : AEnabledWorker
{
    private const uint NodeBroadcast = 0xFFFFFFFF;
    
    private readonly SerialMessageService _serialMessageService;
    private readonly MonitorService _monitorService;
    private readonly string _path;
    private readonly int _speed;
    private readonly double _latitude, _longitude;
    private readonly int _altitude;
    private readonly bool _echoEnabled, _bbsEnabled;
    private readonly int _currentYear;
    private readonly TimeSpan _intervalTelemetryEnvironment, _intervalTelemetryPowerMetric, _intervalPosition;
    private readonly TimeSpan _intervalBbsReminder;
    private readonly TimeSpan _checkConnectionInterval;
    private readonly DataContext _context;
    private readonly SerialConnection _deviceConnection;

    private DateTime? _lastSerialReceived;

    public MeshtasticApp(ILogger<MeshtasticApp> logger, IServiceProvider serviceProvider,
        IConfiguration configuration, SerialMessageService serialMessageService, 
        MonitorService monitorService, IDbContextFactory<DataContext> contextFactory) : base(logger, serviceProvider, false)
    {
        _serialMessageService = serialMessageService;
        _monitorService = monitorService;
        _context = contextFactory.CreateDbContext();
        
        _checkConnectionInterval = TimeSpan.FromMinutes(5);
        
        _currentYear = configuration.GetValueOrThrow<int>("CurrentYear");
        
        var configurationSection = configuration.GetSection("Meshtastic");
        _path = configurationSection.GetValueOrThrow<string>("Path");
        _speed = configurationSection.GetValue("Speed", 38400);
        
        _intervalPosition = TimeSpan.FromMinutes(configurationSection.GetValue("PositionInterval", 60));
        _intervalTelemetryEnvironment = TimeSpan.FromMinutes(configurationSection.GetValue("TelemetryEnvironmentInterval", 60));
        _intervalTelemetryPowerMetric = TimeSpan.FromMinutes(configurationSection.GetValue("TelemetryPowerMetricInterval", 60));
        
        _echoEnabled = configurationSection.GetValue("EchoEnabled", false);
        
        var positionSection = configuration.GetSection("Position");
        _latitude = positionSection.GetValueOrThrow<double>("Latitude");
        _longitude = positionSection.GetValueOrThrow<double>("Longitude");
        _altitude = positionSection.GetValueOrThrow<int>("Altitude");
        
        var bbsSection = configurationSection.GetSection("Bbs");
        _bbsEnabled = bbsSection.GetValue("Enabled", false);
        _intervalBbsReminder = TimeSpan.FromHours(configurationSection.GetValue("ReminderInterval", 1));
        
        _deviceConnection = new SerialConnection(Logger, _path, _speed);
    }

    protected override async Task Start(CancellationToken cancellationToken)
    {   
        await _deviceConnection.Start();

        if (_intervalPosition.TotalSeconds > 0)
        {
            AddDisposable(
                Scheduler.SchedulePeriodic(_intervalPosition, SendWeather)
            );
        }

        if (_intervalTelemetryEnvironment.TotalSeconds > 0)
        {
            AddDisposable(
                Scheduler.SchedulePeriodic(_intervalTelemetryEnvironment, SendPower)
            );
        }

        if (_intervalTelemetryPowerMetric.TotalSeconds > 0)
        {
            AddDisposable(
                Scheduler.SchedulePeriodic(_intervalTelemetryPowerMetric, SendPosition)
            );
        }

        AddDisposable(
            // ReSharper disable once AsyncVoidLambda
            Scheduler.SchedulePeriodic(_checkConnectionInterval, async () =>
            {
                if (!_lastSerialReceived.HasValue || DateTime.UtcNow - _lastSerialReceived >= _checkConnectionInterval)
                {
                    Logger.LogWarning("No Serial received from Meshtastic so reload Worker");

                    await Stop();
                    await Start(cancellationToken);
                }
            })
        );

        _deviceConnection.MessageRecieved += PacketReceived;
    }

    protected override Task Stop()
    {
        try
        {
            _deviceConnection?.Disconnect();
        }
        catch (Exception e)
        {
            Logger.LogWarning(e, "Impossible to disconnect Meshtastic Serial Port");
        }

        return base.Stop();
    }

    private void PacketReceived(object? sender, MessageRecievedEventArgs e)
    {
        _lastSerialReceived = DateTime.UtcNow;
        
        Logger.LogDebug("Meshtastic serial port receive {packet} and payload {payload}", JsonSerializer.Serialize(e.Message), e.Message.GetPayload());

        var myNodeNum = e.DeviceStateContainer.MyNodeInfo.MyNodeNum;
        
        var nodes = e.DeviceStateContainer.Nodes
            .Where(n => n.Num != myNodeNum)
            .Select(n => new MeshtasticNode(n))
            .ToList();
        
        _monitorService.UpdateMeshtasticNodesOnlines(nodes);

        if (DateTime.UtcNow.Year == _currentYear && nodes.Any(n => n.LastHeardFromNow == TimeSpan.Zero))
        {
            SendPosition();
        }

        var packet = e.Message.Packet;
        
        if (packet?.Decoded == null)
        {
            return;
        }
        
        if (packet.From == myNodeNum)
        {
            return;
        }

        var payload = packet.GetPayload();

        Logger.LogInformation("Receiving Meshtastic : {message} on channel {channel} from {from} to {to}", payload, packet.Channel, _deviceConnection?.DeviceStateContainer.GetNodeDisplayName(packet.From), _deviceConnection?.DeviceStateContainer.GetNodeDisplayName(packet.To));

        _monitorService.AddLoRaMeshtasticMessage(e.Message.Packet, e.DeviceStateContainer.GetNodeDisplayName(packet.From), false);
            
        if (!DoEcho(e.Message))
        {
            if (!DoCommand(e.Message))
            {
                DoBbs(e.Message);
            }
        }
    }

    private bool DoEcho(FromRadio radio)
    {
        if (!_echoEnabled)
        {
            return false;
        }
        
        if (_deviceConnection == null)
        {
            throw new Exception("No connection");
        }
        
        var packet = radio.Packet;
        var packetDecoded = packet.Decoded;

        if (packet.To != _deviceConnection.DeviceStateContainer.MyNodeInfo.MyNodeNum 
            || packetDecoded is not { Portnum: PortNum.TextMessageApp })
        {
            return false;
        }
        
        var payloadString = packetDecoded.Payload.ToStringUtf8();

        if (!payloadString.StartsWith('?'))
        {
            return false;
        }
        
        var textMessageFactory = new TextMessageFactory(_deviceConnection.DeviceStateContainer);
        var textMessagePacket = textMessageFactory.CreateTextMessagePacket($">{payloadString}");
        textMessagePacket.To = packet.From;
        textMessagePacket.Channel = packet.Channel;
        textMessagePacket.HopLimit = GetHopLimitForResponse(packet.HopStart, packet.HopLimit);

        Logger.LogInformation("Sending echo message {message}", payloadString);

        Send(textMessagePacket);

        return true;
    }

    private bool DoBbs(FromRadio radio)
    {
        if (!_bbsEnabled)
        {
            return false;
        }
        
        if (_deviceConnection == null)
        {
            throw new Exception("No connection");
        }
        
        var packet = radio.Packet;
        var packetDecoded = packet.Decoded;

        var textMessageFactory = new TextMessageFactory(_deviceConnection.DeviceStateContainer);
        MeshPacket? textMessagePacket;
        
        string? payloadAnswer;

        var bbsMessages = _context.BbsMessages.Where(a =>
                a.To == radio.Packet.From.ToHexString() && !a.ReadAt.HasValue)
            .OrderBy(a => a.CreatedAt)
            .ToList();

        if (packet.To != _deviceConnection.DeviceStateContainer.MyNodeInfo.MyNodeNum 
            || packetDecoded is not { Portnum: PortNum.TextMessageApp })
        {
            if (packet.To != NodeBroadcast)
            {
                return false;
            }
            
            if (!bbsMessages.Any(m => DateTime.UtcNow - m.RemindedAt < _intervalBbsReminder))
            {
                return false;
            }
            
            Logger.LogInformation("There are messages in BBS for {node} so we send it a remind", packet.From.ToHexString());

            payloadAnswer = $"Msg ({bbsMessages.Count})";

            var i = 1;
            foreach (var message in bbsMessages)
            {
                payloadAnswer += $"\n:l {i++} -> {message.From} - {message.CreatedAt.ToFrench()}";
                message.RemindedAt = DateTime.UtcNow;
                _context.Update(message);
            }
            
            _context.SaveChanges();
            
            textMessagePacket = textMessageFactory.CreateTextMessagePacket(payloadAnswer);
            textMessagePacket.To = packet.From;
            textMessagePacket.Channel = packet.Channel;
            textMessagePacket.WantAck = false;
            textMessagePacket.HopLimit = GetHopLimitForResponse(packet.HopStart, packet.HopLimit);
        
            Send(textMessagePacket);
            
            return true;
        }
        
        var payloadString = packetDecoded.Payload.ToStringUtf8();

        if (!payloadString.StartsWith(':'))
        {
            return false;
        }
        
        Logger.LogInformation("Receive BBS meshtastic command from {node} : {command}", packet.From.ToHexString(), payloadString);

        var stringsSplitedOnSpaces = payloadString.Split(" ");
        
        switch (payloadString.TrimStart(':').First())
        {
            case 'r':
                payloadAnswer = $"Msg ({bbsMessages.Count})";

                var i = 1;
                foreach (var message in bbsMessages)
                {
                    payloadAnswer += $"\n:l {i++} -> {message.From} - {message.CreatedAt.ToFrench()}";
                }

                break;
            case 'e':
                var nodeIdString = stringsSplitedOnSpaces.Skip(1).Take(1).FirstOrDefault();
                if (string.IsNullOrWhiteSpace(nodeIdString) || !nodeIdString.StartsWith('!'))
                {
                    payloadAnswer = "KO node id";
                    break;
                }

                var originalMessage = stringsSplitedOnSpaces.Skip(2).JoinString(" ");
                
                var messageToSend = new BbsMessage
                {
                    From = radio.Packet.From.ToHexString(),
                    To = nodeIdString,
                    Message = originalMessage,
                    RemindedAt = DateTime.UtcNow
                };
                
                Logger.LogDebug("Meshtastic bbs original message {message} from {from}", originalMessage, messageToSend.From);
                Logger.LogInformation("Sending meshtastic bbs message #{id} to {to}", messageToSend.Id, messageToSend.To);
                
                _context.Add(messageToSend);
                
                // var bbsMessagesToTo = _context.BbsMessages.Where(a =>
                //         a.To == messageToSend.To && !a.ReadAt.HasValue)
                //     .ToList();
                //
                // foreach (var bbsMessageToTo in bbsMessagesToTo)
                // {
                //     bbsMessageToTo.ReadAt = DateTime.UtcNow; // Not good if we have multiple unread message for one user 
                //     _context.Update(bbsMessageToTo);
                // }
                
                _context.SaveChanges();
                
                var textMessagePacketToSend = textMessageFactory.CreateTextMessagePacket($"BSS: De {messageToSend.From}:\n{messageToSend.Message}\n\n:e {messageToSend.From} TEXTE -> répondre");
                textMessagePacketToSend.To = nodeIdString.ToInteger();
                textMessagePacketToSend.Channel = packet.Channel;
                textMessagePacketToSend.HopLimit = GetHopLimitForResponse(packet.HopStart, packet.HopLimit);
        
                Send(textMessagePacketToSend);

                payloadAnswer = "OK envoyé";
                
                break;
            case 'l':
                var indexString = stringsSplitedOnSpaces.Last();
                if (!int.TryParse(indexString, out var index))
                {
                    payloadAnswer = "KO index";
                    Logger.LogWarning("BBS Meshtastic read message KO. Index parse {index}", indexString);
                    break;
                }

                index--;
                    
                if (index < 0 || index >= bbsMessages.Count)
                {
                    payloadAnswer = "KO index impossible";
                    Logger.LogWarning("BBS Meshtastic read message KO. Not in range {index}/{count}", index, bbsMessages.Count);
                    break;
                }

                var messageToRead = bbsMessages.Skip(index).Take(1).FirstOrDefault();

                if (messageToRead == null)
                {
                    payloadAnswer = "KO message introuvable";
                    Logger.LogWarning("BBS Meshtastic read message KO. Message not found {index}", index);
                    break;
                }
                
                payloadAnswer = $"{messageToRead.From} - {messageToRead.CreatedAt.ToFrench()}\n{messageToRead.Message}\n\n:e {messageToRead.From} TEXTE -> répondre";

                messageToRead.ReadAt = DateTime.UtcNow;
                        
                _context.Update(messageToRead);
                _context.SaveChanges();

                break;
            default:
                payloadAnswer = "BBS cmd:\n:r -> Voir mes messages\n:e !NODEID Ici le message -> Écrire\n:l INDEX -> Lire";
                break;
        }

        textMessagePacket = textMessageFactory.CreateTextMessagePacket($">{payloadAnswer}");
        textMessagePacket.To = packet.From;
        textMessagePacket.Channel = packet.Channel;
        textMessagePacket.HopLimit = GetHopLimitForResponse(packet.HopStart, packet.HopLimit);
        
        Logger.LogInformation("Sending answer for {command} : {answs}", payloadString, payloadAnswer);

        Send(textMessagePacket);

        return true;
    }
    
    private bool DoCommand(FromRadio radio)
    {
        if (_deviceConnection == null)
        {
            throw new Exception("No connection");
        }
        
        var packet = radio.Packet;
        var packetDecoded = packet.Decoded;

        if (packet.To != _deviceConnection.DeviceStateContainer.MyNodeInfo.MyNodeNum 
            || packetDecoded is not { Portnum: PortNum.TextMessageApp })
        {
            return false;
        }
        
        var payloadString = packetDecoded.Payload.ToStringUtf8();

        if (!payloadString.StartsWith('!'))
        {
            return false;
        }

        if (_serialMessageService.SendCommand(payloadString.TrimStart('!')))
        {
            payloadString += " OK";
        }
        else
        {
            payloadString += " KO";
        }
        
        var textMessageFactory = new TextMessageFactory(_deviceConnection.DeviceStateContainer);
        var textMessagePacket = textMessageFactory.CreateTextMessagePacket($">{payloadString}");
        textMessagePacket.To = packet.From;
        textMessagePacket.Channel = packet.Channel;
        textMessagePacket.HopLimit = GetHopLimitForResponse(packet.HopStart, packet.HopLimit);
        
        Logger.LogInformation("Sending ack command for {command}", payloadString);

        Send(textMessagePacket);

        return true;
    }

    private void SendWeather()
    {
        if (_deviceConnection == null)
        {
            throw new Exception("No connection");
        }
        
        var messageFactory = new TelemetryMessageFactory(_deviceConnection.DeviceStateContainer, 0xffffffff);
        
        var telemetryPacket = messageFactory.CreateTelemetryPacket();
        telemetryPacket.WantAck = false;
        telemetryPacket.Priority = MeshPacket.Types.Priority.Background;
        telemetryPacket.Decoded.WantResponse = false;
        telemetryPacket.HopLimit = 0;
        telemetryPacket.Decoded.Payload = new Telemetry
        {
            Time = (uint)DateTime.UtcNow.ToFrench().ToUnixTimestamp(),
            EnvironmentMetrics = new EnvironmentMetrics
            {
                Temperature = EntitiesManagerService.Entities.WeatherTemperature.Value,
                RelativeHumidity = EntitiesManagerService.Entities.WeatherHumidity.Value,
                BarometricPressure = EntitiesManagerService.Entities.WeatherPressure.Value,
                Voltage = EntitiesManagerService.Entities.BatteryVoltage.Value / 1000.0f,
                Current = EntitiesManagerService.Entities.MpptChargeCurrent.Value
            },
        }.ToByteString();
        
        Logger.LogInformation("Sending environment telemetry message");
        
        Send(telemetryPacket);
    }

    private void SendPower()
    {
        if (_deviceConnection == null)
        {
            throw new Exception("No connection");
        }
        
        var messageFactory = new TelemetryMessageFactory(_deviceConnection.DeviceStateContainer, 0xffffffff);
        
        var telemetryPacket = messageFactory.CreateTelemetryPacket();
        telemetryPacket.WantAck = false;
        telemetryPacket.Priority = MeshPacket.Types.Priority.Background;
        telemetryPacket.Decoded.WantResponse = false;
        telemetryPacket.HopLimit = 0;
        telemetryPacket.Decoded.Payload = new Telemetry
        {
            Time = (uint)DateTime.UtcNow.ToFrench().ToUnixTimestamp(),
            PowerMetrics = new PowerMetrics
            {
                Ch1Current = EntitiesManagerService.Entities.BatteryCurrent.Value,
                Ch1Voltage = EntitiesManagerService.Entities.BatteryVoltage.Value / 1000.0f,
                Ch2Current = EntitiesManagerService.Entities.SolarCurrent.Value,
                Ch2Voltage = EntitiesManagerService.Entities.SolarVoltage.Value / 1000.0f,
            }
        }.ToByteString();
        
        Logger.LogInformation("Sending power telemetry message");
        
        Send(telemetryPacket);
    }

    private void SendPosition()
    {
        if (_deviceConnection == null)
        {
            throw new Exception("No connection");
        }
        
        var positionMessageFactory = new PositionMessageFactory(_deviceConnection.DeviceStateContainer);

        decimal divisor = new(1e-7);
        var position = new Position
        {
            LatitudeI = decimal.ToInt32((decimal)(_latitude / (double)divisor)),
            LongitudeI = decimal.ToInt32((decimal)(_longitude / (double)divisor)),
            Altitude = _altitude,
            Time = (uint)DateTime.UtcNow.ToFrench().ToUnixTimestamp()
        };

        var positionPacket = positionMessageFactory.CreatePositionPacket(position);
        positionPacket.WantAck = false;
        positionPacket.Priority = MeshPacket.Types.Priority.Background;
        positionPacket.HopLimit = 0;
        
        Logger.LogInformation("Setting position and time");

        Send(positionPacket);
    }

    private void Send(MeshPacket packet)
    {
        if (_deviceConnection == null)
        {
            throw new Exception("No connection");
        }

        packet.Id = (uint) Random.Shared.NextInt64(0, uint.MaxValue);
        Logger.LogInformation("Sending {message} on channel {channel} to {to}", packet.GetPayload(), packet.Channel, _deviceConnection.DeviceStateContainer.GetNodeDisplayName(packet.To));

        var packetRoRadio = _deviceConnection.ToRadioFactory.CreateMeshPacketMessage(packet);
        _deviceConnection.Send(packetRoRadio);
        
        _monitorService.AddLoRaMeshtasticMessage(packetRoRadio.Packet, _deviceConnection.DeviceStateContainer.GetNodeDisplayName(_deviceConnection.DeviceStateContainer.MyNodeInfo.MyNodeNum), true);
    }

    public override async ValueTask DisposeAsync()
    {
        await Stop();
        GC.SuppressFinalize(this);
    }
    
    private uint GetHopLimitForResponse(uint hopStart, uint hopLimit)
    {
        uint hopMax = 7;
        
        if (_deviceConnection?.DeviceStateContainer.LocalConfig.Lora.HopLimit != null)
        {
            hopMax = _deviceConnection.DeviceStateContainer.LocalConfig.Lora.HopLimit;
        }
        
        if (hopStart != 0)
        {
            // Hops used by the request. If somebody in between running modified firmware modified it, ignore it
            var hopsUsed = hopStart < hopLimit ? hopMax : hopStart - hopLimit;
            if (hopsUsed > hopMax) 
            {
                return hopsUsed; // If the request used more hops than the limit, use the same amount of hops
            }

            if (hopsUsed + 2 < hopMax) 
            {
                return hopsUsed + 2; // Use only the amount of hops needed with some margin as the way back may be different
            }
        }
        
        return hopMax; // Use the default hop limit
    }
}