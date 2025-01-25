using System.Reactive.Concurrency;
using System.Reactive.Linq;
using Monitor.Models;
using Monitor.Services;

namespace Monitor.Workers;

public class GpioApp : AWorker
{
    public static readonly ConfigEntity<bool> StartWifiTurnOn = new("gpio/start_wifi_turn_on", true);
    public static readonly ConfigEntity<bool> StartNprTurnOn = new("gpio/start_npr_turn_on", true);
    public static readonly ConfigEntity<bool> StartMeshtasticTurnOn = new("gpio/start_meshtastic_turn_on", true, true);
    
    public static readonly ConfigEntity<TimeSpan> WifiNprTimeOn = new("low_battery/wifi_npr_time_on", true, TimeSpan.Zero);
    public static readonly ConfigEntity<TimeSpan> WifiNprTimeOff = new("low_battery/wifi_npr_time_off", true, TimeSpan.Zero);
    
    private IDisposable? _schedulerIntervalWifiNpr;
    
    private readonly IScheduler _scheduler;
    private readonly SerialMessageService _serialMessageService;

    public GpioApp(ILogger<GpioApp> logger, IServiceProvider serviceProvider) 
        : base(logger, serviceProvider)
    {
        _scheduler = Services.GetRequiredService<IScheduler>();;
        _serialMessageService = Services.GetRequiredService<SerialMessageService>();

        EntitiesManagerService.Add(StartWifiTurnOn);
        EntitiesManagerService.Add(StartNprTurnOn);
        EntitiesManagerService.Add(StartMeshtasticTurnOn);
        
        EntitiesManagerService.Add(WifiNprTimeOn);
        EntitiesManagerService.Add(WifiNprTimeOff);
    }

    protected override Task Start(CancellationToken cancellationToken)
    {
        AddDisposable(EntitiesManagerService.Entities.GpioWifi.ValueToChange()
            .Do(v => Logger.LogDebug("GPIO WiFi => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetWifi)
        );

        AddDisposable(EntitiesManagerService.Entities.GpioNpr.ValueToChange()
            .Do(v => Logger.LogDebug("GPIO NPR => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetNpr)
        );
        
        AddDisposable(EntitiesManagerService.Entities.GpioMeshtastic.ValueToChange()
            .Do(v => Logger.LogDebug("GPIO Meshtastic => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetMeshtastic)
        );

        Observable.Timer(TimeSpan.FromSeconds(2)).Take(1).Select(_ => true)
            .Merge(
                EntitiesManagerService.Entities.McuStatus.ValueChanges()
                    .Select(v => v.value)
                    .Where(v => v is "starting" or "started")
                    .Select(_ => true)
                    .Do(v => Logger.LogInformation("Will switch GPIO because MCU in state {state}", v))
                    .Delay(TimeSpan.FromSeconds(30)) // Wait init done
            )
            .Merge(StartWifiTurnOn.ValueChanges().Where(s => s.value).Select(v => v.value))
            .Merge(StartNprTurnOn.ValueChanges().Where(s => s.value).Select(v => v.value))
            .Merge(StartMeshtasticTurnOn.ValueChanges().Where(s => s.value).Select(v => v.value))
            .Subscribe(_ => 
            {
                Logger.LogInformation("Switch GPIO with start values");
        
                if (StartWifiTurnOn.Value != EntitiesManagerService.Entities.GpioWifi.Value)
                {
                    EntitiesManagerService.Entities.GpioWifi.Value = StartWifiTurnOn.Value;
                }
        
                if (StartNprTurnOn.Value != EntitiesManagerService.Entities.GpioNpr.Value)
                {
                    EntitiesManagerService.Entities.GpioNpr.Value = StartNprTurnOn.Value;
                }
        
                if (StartMeshtasticTurnOn.Value != EntitiesManagerService.Entities.GpioMeshtastic.Value)
                {
                    EntitiesManagerService.Entities.GpioMeshtastic.Value = StartMeshtasticTurnOn.Value;
                }
            });
        
        AddDisposable(
            WifiNprTimeOn.ValueChanges().Select(_ => true)
                .Merge(WifiNprTimeOff.ValueChanges().Select(_ => true))
                .Merge(EntitiesManagerService.Entities.GpioWifi.ValueChanges().Select(_ => true))
                .Merge(EntitiesManagerService.Entities.GpioNpr.ValueChanges().Select(_ => true))
                .Subscribe(_ =>
                {
                    ReScheduleIntervalWifiNpr();
                }));
        
        ReScheduleIntervalWifiNpr();
        
        return Task.CompletedTask;
    }
    
    private void ReScheduleIntervalWifiNpr()
    {
        _schedulerIntervalWifiNpr?.Dispose();

        if (WifiNprTimeOn.Value == TimeSpan.Zero || WifiNprTimeOff.Value == TimeSpan.Zero)
        {
            return;
        }
        
        if (EntitiesManagerService.Entities.GpioWifi.IsTrue() || EntitiesManagerService.Entities.GpioNpr.IsTrue())
        {
            Logger.LogTrace("Schedule Wifi NPR off in {time}", WifiNprTimeOn.Value);
            
            _schedulerIntervalWifiNpr = _scheduler.Schedule(WifiNprTimeOn.Value, () =>
            {
                Logger.LogInformation("Turn Wifi NPR off because time out {time}", WifiNprTimeOn.Value);
                
                if (StartWifiTurnOn.IsTrue())
                {
                    EntitiesManagerService.Entities.GpioWifi.SetValue(false);
                }

                if (StartNprTurnOn.IsTrue())
                {
                    EntitiesManagerService.Entities.GpioNpr.SetValue(false, true);
                }
            });   
        }
        else
        {
            Logger.LogTrace("Schedule Wifi NPR on in {time}", WifiNprTimeOff.Value);
                
            _schedulerIntervalWifiNpr = _scheduler.Schedule(WifiNprTimeOff.Value, () =>
            {
                Logger.LogInformation("Turn Wifi NPR on because time out {time}", WifiNprTimeOff.Value);
                    
                if (StartWifiTurnOn.IsTrue())
                {
                    EntitiesManagerService.Entities.GpioWifi.SetValue(true);
                }
                
                if (StartNprTurnOn.IsTrue())
                {
                    EntitiesManagerService.Entities.GpioNpr.SetValue(true, true);
                }
            });
        }
    }
}