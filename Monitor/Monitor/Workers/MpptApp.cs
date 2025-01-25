using System.Reactive.Linq;
using Monitor.Extensions;
using Monitor.Models;
using Monitor.Services;

namespace Monitor.Workers;

public class MpptApp : AWorker
{
    private readonly SystemService _systemService;
    private readonly SerialMessageService _serialMessageService;

    public MpptApp(ILogger<MpptApp> logger, IServiceProvider serviceProvider) : base(logger, serviceProvider)
    {
        _systemService = Services.GetRequiredService<SystemService>();
        _serialMessageService = Services.GetRequiredService<SerialMessageService>();
    }

    protected override Task Start(CancellationToken cancellationToken)
    {
        var powerOnVoltageConfigEntity = EntitiesManagerService.Entities.MpptPowerOnVoltage;
        var powerOffVoltageConfigEntity = EntitiesManagerService.Entities.MpptPowerOffVoltage;

        AddDisposable(EntitiesManagerService.Entities.MpptAlertShutdown.ValueChanges()
            .Select(v => v.value)
            .SubscribeAsync(async _ =>
            {
                Logger.LogInformation("Alert so shutdown");
                await _systemService.Shutdown();
            }));

        AddDisposable(EntitiesManagerService.Entities.McuStatus.ValueChanges()
            .Where(v => v.value?.Contains("Restart", StringComparison.CurrentCultureIgnoreCase) ?? false)
            .SubscribeAsync(async _ =>
            {
                Logger.LogInformation("Received MCU Watchdog trigger warning so shutdown");
                
                await _systemService.Shutdown();
            }));

        AddDisposable(powerOffVoltageConfigEntity.ValueToChange()
            .Merge(powerOnVoltageConfigEntity.ValueToChange())
            .Subscribe(_ =>
            {
                _serialMessageService.SetPowerOnOffVoltage(
                    powerOnVoltageConfigEntity.Value,
                    powerOffVoltageConfigEntity.Value
                );
            }));
        
        return Task.CompletedTask;
    }
}