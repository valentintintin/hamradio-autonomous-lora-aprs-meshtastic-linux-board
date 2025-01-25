using System.Reactive.Linq;
using Monitor.Services;

namespace Monitor.Workers;

public class McuFeatureApp : AWorker
{
    private readonly SerialMessageService _serialMessageService;

    public McuFeatureApp(ILogger<McuFeatureApp> logger, IServiceProvider serviceProvider) 
        : base(logger, serviceProvider)
    {
        _serialMessageService = Services.GetRequiredService<SerialMessageService>();
    }

    protected override Task Start(CancellationToken cancellationToken)
    {
        AddDisposable(EntitiesManagerService.Entities.FeatureMpptWatchdogSafetyEnabled.ValueToChange()
            .Do(v => Logger.LogDebug("Watchdog Safety => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetWatchdogSafety)
        );
        
        AddDisposable(EntitiesManagerService.Entities.FeatureAprsDigipeaterEnabled.ValueToChange()
            .Do(v => Logger.LogDebug("APRS DigiPeater => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetAprsDigipeater)
        );
        
        AddDisposable(EntitiesManagerService.Entities.FeatureAprsTelemetryEnabled.ValueToChange()
            .Do(v => Logger.LogDebug("APRS Telemetry => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetAprsTelemetry)
        );
        
        AddDisposable(EntitiesManagerService.Entities.FeatureAprsPositionEnabled.ValueToChange()
            .Do(v => Logger.LogDebug("APRS Position => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetAprsPosition)
        );

        AddDisposable(EntitiesManagerService.Entities.FeatureSleepEnabled.ValueToChange()
            .Do(v => Logger.LogDebug("Sleep => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetSleep)
        );

        AddDisposable(EntitiesManagerService.Entities.FeatureResetOnErrorEnabled.ValueToChange()
            .Do(v => Logger.LogDebug("Reset on error => {value}", v))
            .Select(v => v.valueToChange)
            .Subscribe(_serialMessageService.SetResetOnError)
        );

        return Task.CompletedTask;
    }
}