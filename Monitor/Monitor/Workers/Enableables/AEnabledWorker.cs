using System.Reactive.Linq;
using Monitor.Extensions;
using Monitor.Models;

namespace Monitor.Workers.Enableables;

public abstract class AEnabledWorker : AWorker
{
    public readonly ConfigEntity<bool> Enabled;
    
    protected AEnabledWorker(ILogger<AEnabledWorker> logger, IServiceProvider serviceProvider, bool enabled = true) : base(logger, serviceProvider)
    {
        Enabled = new ConfigEntity<bool>($"worker/{GetType().Name.ToLower().Replace("app", "")}", true, enabled);
        EntitiesManagerService.Add(Enabled);
    }

    public override Task StartAsync(CancellationToken cancellationToken)
    {
        Observable.Timer(TimeSpan.Zero, RetryDuration).Select(_ => Enabled.Value)
            .Merge(Enabled.ValueChanges().Select(v => v.value))
            .SubscribeAsync(async state =>
            {
                try
                {
                    await DoState(state, cancellationToken);
                }
                catch (Exception e)
                {
                    try
                    {
                        await Stop();
                    }
                    catch (Exception ee)
                    {
                        Logger.LogWarning(ee, "Crash during stop of worker after crash");
                    }
                    
                    Logger.LogCritical(e, "Crash of worker, retry in {duration}", RetryDuration);
                }
            });

        return Task.CompletedTask;
    }

    private async Task DoState(bool state, CancellationToken cancellationToken)
    {
        if (state)
        {
            if (Started)
            {
                return;
            }

            Logger.LogInformation("Starting {worker}", Enabled.Id);

            await Start(cancellationToken);
            Started = true;

            Logger.LogInformation("{worker} started !", Enabled.Id);
        }
        else
        {
            if (!Started)
            {
                return;
            }

            Logger.LogInformation("Stopping {worker}", Enabled.Id);

            await Stop();

            Logger.LogInformation("{worker} stopped !", Enabled.Id);
        }
    }
}