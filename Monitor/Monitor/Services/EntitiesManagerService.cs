using System.Reactive.Concurrency;
using System.Reactive.Linq;
using System.Reflection;
using Microsoft.EntityFrameworkCore;
using Monitor.Context;
using Monitor.Context.Entities;
using Monitor.Extensions;
using Monitor.Models;
using MQTTnet;
using MQTTnet.Client;
using MQTTnet.Formatter;

namespace Monitor.Services;

public class EntitiesManagerService : BackgroundService
{
    private readonly ILogger<EntitiesManagerService> _logger;
    public static MqttEntities Entities { get; } = new();

    private readonly string _topicBase;
    private readonly string _clientId;
    private readonly IConfigurationSection _configurationSection;
    private readonly IMqttClient _mqttClient;
    private readonly DataContext _context;
    private readonly IScheduler _scheduler;

    public EntitiesManagerService(ILogger<EntitiesManagerService> logger, 
        IConfiguration configuration, IServiceProvider serviceProvider,
        IDbContextFactory<DataContext> contextFactory)
    {
        _logger = logger;
        _configurationSection = configuration.GetSection("Mqtt");
        _topicBase = _configurationSection.GetValueOrThrow<string>("TopicBase");
        _clientId = _configurationSection.GetValueOrThrow<string>("ClientId");
        _mqttClient = new MqttFactory().CreateMqttClient();
        _context = contextFactory.CreateDbContext();
        _scheduler = serviceProvider.CreateScope().ServiceProvider.GetRequiredService<IScheduler>();
        
        foreach (var property in typeof(MqttEntities).GetProperties())
        {
            var entity = (IStringConfigEntity?)property.GetValue(Entities);
            Add(entity ?? throw new InvalidOperationException());
        }

        _mqttClient.ConnectedAsync += async _ =>
        {
            _logger.LogInformation("Connection successful to MQTT");
            
            await _mqttClient.SubscribeAsync(
                new MqttTopicFilterBuilder()
                    .WithTopic($"{_topicBase}/#")
                    .WithNoLocal()
                    .Build()
            );
        };

        _mqttClient.DisconnectedAsync += async args => 
        {
            _logger.LogWarning(args.Exception, "MQTT disconnected");
            
            await Task.Delay(TimeSpan.FromMinutes(1));
            await ConnectMqtt();
        };
    }

    public async Task ConnectMqtt()
    {
        _logger.LogInformation("Run connection to MQTT");
        
        await _mqttClient.ConnectAsync(new MqttClientOptionsBuilder()
            .WithTcpServer(_configurationSection.GetValueOrThrow<string>("Host"), _configurationSection.GetValue("Port", 1883))
            .WithKeepAlivePeriod(TimeSpan.FromMinutes(5))
            .WithClientId(_clientId)
            .WithProtocolVersion(MqttProtocolVersion.V500)
            .Build());
    }

    public void Add(IStringConfigEntity configEntity)
    {
        var config = _context.Configs.FirstOrDefault(c => c.Name == configEntity.Id);
        if (config == null && configEntity.SaveInBdd)
        {
            var value = configEntity.ValueAsString();
            
            _logger.LogTrace("Add entity {entity} with value {value}", configEntity.Id, value);

            config = new Config
            {
                Name = configEntity.Id,
                Value = value
            };
            
            _context.Add(config);
            _context.SaveChanges();
        }

        if (config != null)
        {
            configEntity.SetFromStringPayload(config.Value);
        }
        
        configEntity.ValueStringAsync()
            .SubscribeAsync(async value =>
            {
                _logger.LogTrace("Update {entityId} to {state}", configEntity.Id, value);

                if (config != null)
                {
                    config.Value = value;
                    await _context.SaveChangesAsync();
                }
                
                if (configEntity.SendToMqtt && _mqttClient.IsConnected)
                {
                    _logger.LogTrace("Send MQTT {entityId} to {state}", configEntity.Id, value);

                    var mqttApplicationMessage = new MqttApplicationMessageBuilder()
                        .WithTopic($"{_topicBase}/{configEntity.Id}")
                        .WithPayload(value)
                        .WithRetainFlag(configEntity.SaveInBdd);

                    await _mqttClient.PublishAsync(mqttApplicationMessage.Build());
                }
            });

        if (configEntity.ChangeWithAck)
        {
            IDisposable? disposable = null;
            
            configEntity.ValueToChangeStringAsync().Subscribe(_ =>
            {
                disposable?.Dispose();
                disposable = _scheduler.Schedule(TimeSpan.FromSeconds(5), _ =>
                {
                    configEntity.ClearValueToChange();
                });
            });
        }
    }
    
    protected override async Task ExecuteAsync(CancellationToken cancellationToken)
    {
        await ConnectMqtt();

        cancellationToken.WaitHandle.WaitOne();
        
        if (_mqttClient.IsConnected)
        {
            await _mqttClient.DisconnectAsync();
        }
    }
}

public record MqttEntities
{
    public ConfigEntity<bool> GpioWifi { get; } = new("gpio/wifi", false, false, true, true);
    public ConfigEntity<bool> GpioNpr { get; } = new("gpio/npr", false, false, true, true);
    public ConfigEntity<bool> GpioMeshtastic { get; } = new("gpio/meshtastic", false, false, true, true);
    public ConfigEntity<int> GpioBoxLdr { get; } = new("gpio/box_ldr");

    public ConfigEntity<bool> StatusBoxOpened { get; } = new("status/box_opened", false, default, true);

    public ConfigEntity<string> McuStatus { get; } = new("mcu/status", false, default, true);
    public ConfigEntity<float> TemperatureRtc { get; } = new("mcu/temperature_rtc", false, default, true);
    public ConfigEntity<TimeSpan> McuUptime { get; } = new("mcu/uptime", false, default, true);
    
    public ConfigEntity<bool> FeatureMpptWatchdogSafetyEnabled { get; } = new("feature/mppt_watchdog_safety", true, false, false, true);
    public ConfigEntity<bool> FeatureWatchdogEnabled { get; } = new("feature/watchdog", true, false, false, true);
    public ConfigEntity<bool> FeatureWatchdogLoraTxEnabled { get; } = new("feature/watchdog_lora_tx", true, false, false, true);
    public ConfigEntity<bool> FeatureAprsDigipeaterEnabled { get; } = new("feature/aprs_digipeater", true, true, false, true);
    public ConfigEntity<bool> FeatureAprsTelemetryEnabled { get; } = new("feature/aprs_telemetry", true, true, false, true);
    public ConfigEntity<bool> FeatureAprsPositionEnabled { get; } = new("feature/aprs_position", true, true, false, true);
    public ConfigEntity<bool> FeatureSleepEnabled { get; } = new("feature/sleep", true, false, false, true);
    public ConfigEntity<bool> FeatureResetOnErrorEnabled { get; } = new("feature/reset_on_error", true, false, false, true);

    public ConfigEntity<float> WeatherTemperature { get; } = new("weather/temperature", false, default, true);
    public ConfigEntity<float> WeatherHumidity { get; } = new("weather/humidity", false, default, true);
    public ConfigEntity<float> WeatherPressure { get; } = new("weather/pressure", false, default, true);

    public ConfigEntity<int> BatteryVoltage { get; } = new("battery/voltage", false, default, true);
    public ConfigEntity<int> BatteryCurrent { get; } = new("battery/current", false, default, true);
    public ConfigEntity<int> SolarVoltage { get; } = new("solar/voltage", false, default, true);
    public ConfigEntity<int> SolarCurrent { get; } = new("solar/current", false, default, true);
    public ConfigEntity<bool> SolarIsDay { get; } = new("solar/is_day", false, default, true);

    public ConfigEntity<int> MpptChargeCurrent { get; } = new("mppt/charge_current");
    public ConfigEntity<string> MpptStatus { get; } = new("mppt/status");
    public ConfigEntity<bool> MpptAlertShutdown { get; } = new("mppt/alert_shutdown", false, default, true);
    public ConfigEntity<bool> MpptPowerEnabled { get; } = new("mppt/power_enabled");
    public ConfigEntity<int> MpptPowerOffVoltage { get; } = new("mppt/power_off_voltage", true, 11500, false, true);
    public ConfigEntity<int> MpptPowerOnVoltage { get; } = new("mppt/power_on_voltage", true, 12500, false, true);
    public ConfigEntity<float> MpptTemperature { get; } = new("mppt/temperature", false, default, true);
    
    public ConfigEntity<bool> WatchdogEnabled { get; } = new("watchdog/enabled", false, default, true);
    public ConfigEntity<TimeSpan> WatchdogCounter { get; } = new("watchdog/counter");
    public ConfigEntity<TimeSpan> WatchdogPowerOffTime { get; } = new("watchdog/power_off_time", false, default, true);

    public ConfigEntity<LoRaMessage> LoraAprsTxPayload { get; } = new("lora/aprs/tx_payload", false, default, true);

    public ConfigEntity<LoRaMessage> LoraAprsRxPayload { get; } = new("lora/aprs/rx_payload", false, default, true);
    
    public ConfigEntity<LoRaMessage> LoraMeshtasticTxPayload { get; } = new("lora/meshtastic/tx_payload", false, default, true);

    public ConfigEntity<LoRaMessage> LoraMeshtasticRxPayload { get; } = new("lora/meshtastic/rx_payload", false, default, true);
    public ConfigEntity<List<MeshtasticNode>> LoraMeshtasticNodesOnlines { get; } = new("lora/meshtastic/nodes_onlines", false, default, true);
    
    public ConfigEntity<TimeSpan> SystemUptime { get; } = new("system/uptime", false, default, true);
    public ConfigEntity<int> SystemCpu { get; } = new("system/cpu");
    public ConfigEntity<int> SystemRam { get; } = new("system/ram");
    public ConfigEntity<int> SystemDisk { get; } = new("system/disk", false, default, true);
}