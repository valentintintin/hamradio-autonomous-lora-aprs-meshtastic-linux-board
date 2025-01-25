using System.Text.Encodings.Web;
using System.Text.Json;
using System.Text.Json.Serialization;
using Meshtastic.Extensions;
using Meshtastic.Protobufs;
using Microsoft.EntityFrameworkCore;
using Monitor.Context;
using Monitor.Context.Entities;
using Monitor.Extensions;
using Monitor.Models;
using Monitor.Models.SerialMessages;

namespace Monitor.Services;

public class MonitorService(
    ILogger<MonitorService> logger,
    IDbContextFactory<DataContext> contextFactory,
    SystemService systemService)
    : AService(logger)
{
    public static readonly MonitorState State = new();
    
    private readonly JsonSerializerOptions _jsonOptions = new()
    {
        Encoder = JavaScriptEncoder.UnsafeRelaxedJsonEscaping,
        WriteIndented = true,
        Converters = { new JsonStringEnumConverter() }
    };

    public async Task UpdateStateFromMessage(Message message)
    {
        var context = await contextFactory.CreateDbContextAsync();

        State.LastMessagesReceived.Add(message);
        
        switch (message)
        {
            case McuSystemData systemData:
                Logger.LogInformation("New system data received : {message}", systemData);
                
                State.McuSystem = systemData;
                
                EntitiesManagerService.Entities.McuStatus.SetValue($"{systemData.State} {(systemData.NbError > 0 ? $"({systemData.NbError} errors)" : "")}", true);
                EntitiesManagerService.Entities.StatusBoxOpened.SetValue(systemData.BoxOpened, true);
                EntitiesManagerService.Entities.FeatureMpptWatchdogSafetyEnabled.SetValue(systemData.MpptWatchdogSafetyEnabled, true);
                EntitiesManagerService.Entities.FeatureWatchdogEnabled.SetValue(systemData.Watchdog, true);
                EntitiesManagerService.Entities.FeatureWatchdogLoraTxEnabled.SetValue(systemData.WatchdogLoraTx, true);
                EntitiesManagerService.Entities.FeatureAprsDigipeaterEnabled.SetValue(systemData.AprsDigipeaterEnabled, true);
                EntitiesManagerService.Entities.FeatureAprsTelemetryEnabled.SetValue(systemData.AprsTelemetryEnabled, true);
                EntitiesManagerService.Entities.FeatureAprsPositionEnabled.SetValue(systemData.AprsPositionEnabled, true);
                EntitiesManagerService.Entities.FeatureSleepEnabled.SetValue(systemData.Sleep, true);
                EntitiesManagerService.Entities.FeatureResetOnErrorEnabled.SetValue(systemData.ResetError, true);
                EntitiesManagerService.Entities.TemperatureRtc.SetValue(systemData.TemperatureRtc, true);
                EntitiesManagerService.Entities.McuUptime.SetValue(systemData.UptimeTimeSpan, true);
                
                systemService.SetTime(State.McuSystem.DateTime.DateTime);
                break;
            case WeatherData weatherData:
                Logger.LogInformation("New weather data received : {message}", weatherData);

                State.Weather = weatherData;
                
                EntitiesManagerService.Entities.WeatherTemperature.SetValue(weatherData.Temperature, true);
                EntitiesManagerService.Entities.WeatherHumidity.SetValue(weatherData.Humidity, true);
                EntitiesManagerService.Entities.WeatherPressure.SetValue(weatherData.Pressure, true);

                context.Add(new Weather
                {
                    Temperature = weatherData.Temperature,
                    Humidity = weatherData.Humidity,
                    Pressure = weatherData.Pressure
                });
                await context.SaveChangesAsync();
                break;
            case MpptData mpptData:
                Logger.LogInformation("New MPPT data received : {message}", mpptData);

                State.Mppt = mpptData;
                
                EntitiesManagerService.Entities.BatteryVoltage.SetValue(mpptData.BatteryVoltage, true);
                EntitiesManagerService.Entities.BatteryCurrent.SetValue(mpptData.BatteryCurrent, true);
                EntitiesManagerService.Entities.SolarVoltage.SetValue(mpptData.SolarVoltage, true);
                EntitiesManagerService.Entities.SolarCurrent.SetValue(mpptData.SolarCurrent, true);
                EntitiesManagerService.Entities.MpptChargeCurrent.SetValue(mpptData.CurrentCharge, true);
                EntitiesManagerService.Entities.MpptStatus.SetValue(mpptData.StatusString, true);
                EntitiesManagerService.Entities.SolarIsDay.SetValue(!mpptData.Night, true);
                EntitiesManagerService.Entities.MpptAlertShutdown.SetValue(mpptData.Alert, true);
                EntitiesManagerService.Entities.MpptPowerEnabled.SetValue(mpptData.PowerEnabled, true);
                EntitiesManagerService.Entities.WatchdogEnabled.SetValue(mpptData.WatchdogEnabled, true);
                EntitiesManagerService.Entities.WatchdogCounter.SetValue(TimeSpan.FromSeconds(mpptData.WatchdogCounter), true);
                EntitiesManagerService.Entities.WatchdogPowerOffTime.SetValue(TimeSpan.FromSeconds(mpptData.WatchdogPowerOffTime), true);
                EntitiesManagerService.Entities.MpptPowerOffVoltage.SetValue(mpptData.PowerOffVoltage, true);
                EntitiesManagerService.Entities.MpptPowerOnVoltage.SetValue(mpptData.PowerOnVoltage, true);
                EntitiesManagerService.Entities.MpptTemperature.SetValue(mpptData.Temperature, true);

                context.Add(new Mppt
                {
                    BatteryVoltage = mpptData.BatteryVoltage,
                    BatteryCurrent = mpptData.BatteryCurrent,
                    SolarVoltage = mpptData.SolarVoltage,
                    SolarCurrent = mpptData.SolarCurrent,
                    CurrentCharge = mpptData.CurrentCharge,
                    Status = mpptData.Status,
                    StatusString = mpptData.StatusString,
                    Night = mpptData.Night,
                    Alert = mpptData.Alert,
                    WatchdogEnabled = mpptData.WatchdogEnabled,
                    WatchdogPowerOffTime = mpptData.WatchdogPowerOffTime,
                    Temperature = mpptData.Temperature
                });
                await context.SaveChangesAsync();
                break;
            case GpioData gpioData:
                Logger.LogInformation("New GPIO data received : {message}", gpioData);

                State.Gpio = gpioData;
                
                EntitiesManagerService.Entities.GpioWifi.SetValue(gpioData.Wifi, true);
                EntitiesManagerService.Entities.GpioNpr.SetValue(gpioData.Npr, true);
                EntitiesManagerService.Entities.GpioMeshtastic.SetValue(gpioData.Meshtastic, true);
                EntitiesManagerService.Entities.GpioBoxLdr.SetValue(gpioData.Ldr, true);

                context.Add(new Context.Entities.System
                {
                    Npr = gpioData.Npr,
                    Wifi = gpioData.Wifi,
                    Meshtastic = gpioData.Meshtastic,
                    Uptime = (long)EntitiesManagerService.Entities.SystemUptime.Value.TotalSeconds,
                    BoxOpened = State.McuSystem.BoxOpened,
                    McuUptime = State.McuSystem.Uptime,
                    TemperatureRtc = State.McuSystem.TemperatureRtc,
                    NbError = State.McuSystem.NbError
                });
                await context.SaveChangesAsync();
                break;
            case LoraData loraData:
                Logger.LogInformation("New LoRa APRS data received : {message}", loraData);

                string? sender = null;
                
                try
                {
                    var packet = loraData.Payload.ToAprsPacket();
                    sender = packet.Sender;
                }
                catch (Exception e)
                {
                    Logger.LogWarning(e, "LoRa APRS frame received not decodable : {payload}", loraData.Payload);
                }

                var l = new LoRa
                {
                    Sender = sender,
                    Frame = loraData.Payload,
                    IsTx = loraData.IsTx,
                    IsMeshtastic = false
                };
                context.Add(l);
                await context.SaveChangesAsync();

                var loRaMessage = new LoRaMessage(l);
                
                if (loraData.IsTx)
                {
                    EntitiesManagerService.Entities.LoraAprsTxPayload.SetValue(loRaMessage, true);
                    State.LoRa.AprsTx.Add(loRaMessage);
                }
                else
                {
                    EntitiesManagerService.Entities.LoraAprsRxPayload.SetValue(loRaMessage, true);
                    State.LoRa.AprsRx.Add(loRaMessage);
                }
                
                break;
        }
    }

    public void AddLoRaMeshtasticMessage(MeshPacket packet, string sender, bool isTx) {
        var context = contextFactory.CreateDbContext();

        var json = JsonSerializer.Serialize(new {
            Type = packet.Decoded?.Portnum.ToString(),
            Payload = packet.GetPayload(),
            Raw = packet
        }, _jsonOptions);

        var l = new LoRa
        {
            Frame = json,
            Sender = sender,
            IsTx = isTx,
            IsMeshtastic = true
        };
        context.Add(l);
        context.SaveChanges();

        var loRaMessage = new LoRaMessage(l);
        if (isTx)
        {
            EntitiesManagerService.Entities.LoraMeshtasticTxPayload.SetValue(loRaMessage, true);
            State.LoRa.MeshtasticTx.Add(loRaMessage);
        }
        else
        {
            EntitiesManagerService.Entities.LoraMeshtasticRxPayload.SetValue(loRaMessage, true); 
            State.LoRa.MeshtasticRx.Add(loRaMessage);
        }
    }

    public void UpdateMeshtasticNodesOnlines(List<MeshtasticNode> nodes)
    {
        EntitiesManagerService.Entities.LoraMeshtasticNodesOnlines.SetValue(nodes, true);
        State.LoRa.MeshtasticNodes = nodes;
    }

    public void UpdateSystemState(SystemState? systemState)
    {
        Logger.LogInformation("New system info received");
        
        State.System = systemState;

        if (State.System != null)
        {
            EntitiesManagerService.Entities.SystemUptime.SetValue(State.System.UptimeTimeSpan, true);
            EntitiesManagerService.Entities.SystemCpu.SetValue(State.System.Cpu, true);
            EntitiesManagerService.Entities.SystemRam.SetValue(State.System.Ram, true);
            EntitiesManagerService.Entities.SystemDisk.SetValue(State.System.Disk, true);
        }
    }
}