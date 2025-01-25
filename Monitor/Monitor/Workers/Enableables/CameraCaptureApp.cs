using System.Reactive.Concurrency;
using System.Reactive.Linq;
using AntDesign;
using Monitor.Models;
using Monitor.Services;

namespace Monitor.Workers.Enableables;

public class CameraCaptureApp : AEnabledWorker
{
    private readonly IScheduler _scheduler;
    private readonly INotificationService _notificationService;
    public readonly ConfigEntity<TimeSpan> Interval = new("cameras/interval", true, TimeSpan.FromSeconds(30));
    public readonly ConfigEntity<TimeSpan> IntervalWithMerge = new("cameras/interval_with_merge", true, TimeSpan.FromMinutes(3));

    private readonly CameraService _cameraService;

    private bool _shouldMerge;
    private IDisposable? _schedulerInterval;
    private IDisposable? _schedulerIntervalWithMerge;

    public CameraCaptureApp(ILogger<CameraCaptureApp> logger, IServiceProvider serviceProvider)
        : base(logger, serviceProvider, false)
    {
        _scheduler = Services.GetRequiredService<IScheduler>();
        _cameraService = Services.GetRequiredService<CameraService>();
        _notificationService = Services.GetRequiredService<INotificationService>();
        EntitiesManagerService.Add(Interval);
        EntitiesManagerService.Add(IntervalWithMerge);
    }

    private async Task Do()
    {
        try
        {
            if (_shouldMerge)
            {
                await _cameraService.CaptureAndCreateMergedImage();
                _shouldMerge = false;
            }
            else
            {
                _cameraService.MakeAllImages();
            }

            await _notificationService.Success(new NotificationConfig
            {
                Key = "camera", 
                Message = "Nouvelles images disponibles"
            });
        }
        catch (Exception e)
        {
            Logger.LogError(e, "Error during Camera Capture");

            await _notificationService.Error(new NotificationConfig
            {
                Key = "camera", 
                Message = "Erreur durant la prise de photo",
                Description = e.ToString()
            });
        }
    }

    protected override async Task Start(CancellationToken cancellationToken)
    {
        AddDisposable(
            Interval.ValueChanges()
                .Merge(IntervalWithMerge.ValueChanges())
                .Subscribe(_ =>
                {
                    ReScheduleInterval();
                }));
        
        await Do();
        ReScheduleInterval();
    }

    private void ReScheduleInterval()
    {
        _schedulerInterval?.Dispose();
        _schedulerInterval = _scheduler.SchedulePeriodic(Interval.Value, async () =>
        {
            await Do();
        });
        
        _schedulerIntervalWithMerge?.Dispose();
        _schedulerIntervalWithMerge = _scheduler.SchedulePeriodic(IntervalWithMerge.Value, () =>
        {
            _shouldMerge = true;
        });
    }
}