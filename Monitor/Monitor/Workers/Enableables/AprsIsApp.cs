using System.Globalization;
using System.Reactive.Linq;
using AprsSharp.AprsIsClient;
using AprsSharp.AprsParser;
using AprsSharp.Shared;
using GeoCoordinatePortable;
using Microsoft.EntityFrameworkCore;
using Monitor.Context;
using Monitor.Extensions;
using Monitor.Services;

namespace Monitor.Workers.Enableables;

public class AprsIsApp : AEnabledWorker
{
    private readonly SerialMessageService _serialMessageService;
    private readonly AprsIsClient _aprsIsClient;
    private readonly string _callsign, _passcode, _server, _filter, _destination, _path;
    private readonly string? _objectName, _objectComment;
    private readonly DataContext _context;
    private readonly Position? _objectPosition;
    private readonly TimeSpan? _durationHeard = TimeSpan.FromMinutes(30);
    private readonly TcpConnection _tcpConnection = new();

    public AprsIsApp(ILogger<AprsIsApp> logger, IServiceProvider serviceProvider,
        IConfiguration configuration, IDbContextFactory<DataContext> contextFactory) : base(logger, serviceProvider, false)
    {
        _serialMessageService = Services.GetRequiredService<SerialMessageService>();
        _context = contextFactory.CreateDbContext();
        
        var configurationSection = configuration.GetSection("AprsIs");
        
        _aprsIsClient = new AprsIsClient(Services.GetRequiredService<ILogger<AprsIsClient>>(), _tcpConnection);
        // _aprsIsClient.ReceivedTcpMessage += message => Logger.LogTrace(message);
        if (configurationSection.GetValueOrThrow<bool>("IsToRf"))
        {
            _aprsIsClient.ReceivedPacket += ComputeReceivedPacket;
        }

        var positionSection = configuration.GetSection("Position");
        
        var latitude = positionSection.GetValueOrThrow<double>("Latitude");
        var longitude = positionSection.GetValueOrThrow<double>("Longitude");
        
        _callsign = configurationSection.GetValueOrThrow<string>("Callsign");
        _destination = configurationSection.GetValueOrThrow<string>("Destination");
        _path = configurationSection.GetValueOrThrow<string>("Path");
        _passcode = configurationSection.GetValueOrThrow<string>("Passcode");
        _server = configurationSection.GetValueOrThrow<string>("Server");
        _filter = $"r/{latitude.ToString( CultureInfo.InvariantCulture)}/{longitude.ToString(CultureInfo.InvariantCulture)}/{configurationSection.GetValueOrThrow<int>("RadiusKm")} -e/{_callsign} {configurationSection.GetValueOrThrow<string>("Filter")}\n";

        if (configurationSection.GetValue("AlwaysTx", true))
        {
            _durationHeard = null;
        }

        var beaconObjectSection = configurationSection.GetSection("BeaconObject");
        if (beaconObjectSection.GetValue("Enable", false))
        {
            _objectName = beaconObjectSection.GetValueOrThrow<string>("Name");
            _objectComment = beaconObjectSection.GetValue<string>("Comment");
            _objectPosition = new Position(new GeoCoordinate(latitude, longitude), 
                beaconObjectSection.GetValueOrThrow<char>("SymbolTable"),
                beaconObjectSection.GetValueOrThrow<char>("SymbolCode")
                );
            
            _aprsIsClient.ChangedState += state =>
            {
                if (state == ConnectionState.LoggedIn)
                {
                    SendBeaconObjectPacket();
                }
            };
            
            AddDisposable(Observable.Timer(TimeSpan.FromMinutes(30)).Subscribe(_ =>
            {
                SendBeaconObjectPacket();
            }));
        }
    }

    protected override Task Start(CancellationToken cancellationToken)
    {
        Connect();
        return Task.CompletedTask;
    }

    private void Connect()
    {
        AddDisposable(_aprsIsClient.Receive(_callsign, _passcode, _server, _filter));
    }

    protected override async Task Stop()
    {
        if (_objectPosition != null)
        {
            if (SendBeaconObjectPacket(false))
            {
                await Task.Delay(1500);
            }
        }

        _aprsIsClient.Disconnect();

        await base.Stop();
    }

    private void ComputeReceivedPacket(Packet packet)
    {
        Logger.LogTrace("Received APRS-IS Packet from {from} to {to} of type {type}", packet.Sender, packet.Path.JoinString(), packet.InfoField.Type);

        if (packet.Sender == _callsign)
        {
            Logger.LogDebug("It's our packet so ignored");
            
            return;
        }
        
        if (packet.Path.Contains("?") || packet.Path.Contains("qAX") || packet.Path.Contains("RFONLY") ||
            packet.Path.Contains("NOGATE") || packet.Path.Contains("TCPXX"))
        {
            Logger.LogDebug("Packet from {from} to {to} has not the correct path {path} for RF", packet.Sender, packet.Path.JoinString(), packet.Path);
            
            return;
        }

        if (!HasStationHeard())
        {
            Logger.LogDebug("No station heard since {duration}. So no TX", _durationHeard);
            return;
        }

        try
        {
            var packetToSendTnc2 = $"{_callsign}>{_destination},{_path}:}}{packet.Sender}>{packet.Path.First()},TCPIP,{_callsign}*:{packet.InfoField.Encode()}";
            Logger.LogInformation("Packet ready to be send to RF: {packet}", packetToSendTnc2);

            _serialMessageService.SendLora(packetToSendTnc2);
        }
        catch (Exception e)
        {
            Logger.LogWarning(e, "Impossible to compute packet from {from} to {to} of type {type}", packet.Sender, packet.Destination, packet.InfoField.Type);
        }
    }

    private bool HasStationHeard()
    {
        if (!_durationHeard.HasValue)
        {
            return true;
        }
        
        var lastHeard = DateTime.UtcNow - _durationHeard.Value;
        return _context.LoRas.Any(e => !e.IsTx && e.CreatedAt >= lastHeard && !e.IsMeshtastic);
    }

    private bool SendBeaconObjectPacket(bool alive = true)
    {
        if (_aprsIsClient.State != ConnectionState.LoggedIn)
        {
            Logger.LogWarning("Impossible to send beacon because not logged");

            Connect();
            
            return false;
        }

        var comment = alive ? $"{_objectComment} Up:{EntitiesManagerService.Entities.SystemUptime.Value}" : "Eteint";
        var packet = $"{_callsign}>TCPIP:){_objectName}{(alive ? '!' : '_')}{_objectPosition!.Encode()}{comment}";
        
        Logger.LogInformation("Send packet beacon object alive ? {alive} : {packet}", alive, packet);

        try
        {
            _tcpConnection.SendString(packet + "\n");

            return true;
        }
        catch (Exception e)
        {
            Logger.LogError(e, "Impossible to send beacon");

            return false;
        }
    }
}