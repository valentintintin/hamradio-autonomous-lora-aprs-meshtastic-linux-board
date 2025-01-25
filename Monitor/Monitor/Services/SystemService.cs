using System.Diagnostics;
using Monitor.Extensions;

namespace Monitor.Services;

public class SystemService(ILogger<SystemService> logger, SerialMessageService serialMessageService, IConfiguration configuration)
    : AService(logger)
{
    private TimeSpan? WillSleep { get; set; }
    private readonly int _currentYear = configuration.GetValueOrThrow<int>("CurrentYear");

    public void AskForShutdown(TimeSpan sleepTime)
    {
        Logger.LogInformation("Ask for shutdown during {time}", sleepTime);
        
        WillSleep = sleepTime;

        if (WillSleep.HasValue)
        {
            serialMessageService.SetWatchdog(WillSleep.Value);
        }
    }

    public async Task Shutdown()
    {
        Logger.LogInformation("Send shutdown command");

        await Task.Delay(2500);
        
        await File.WriteAllTextAsync("/proc/sys/kernel/sysrq", "1");
        await File.WriteAllTextAsync("/proc/sysrq-trigger", "s");
        await File.WriteAllTextAsync("/proc/sysrq-trigger", "u");
        await File.WriteAllTextAsync("/proc/sysrq-trigger", "o");
    }

    public void SetTime(DateTime dateTime)
    {
        if (Math.Abs((DateTime.UtcNow - dateTime).TotalMinutes) < 1)
        {
            Logger.LogDebug("Change dateTime not done because difference is < 1 minute : {now} and {new}", DateTime.UtcNow, dateTime);
            return;
        }
        
        if (dateTime.Year != _currentYear)
        {
            Logger.LogWarning("Change dateTime impossible because incoherent {dateTime}", dateTime);
            return;
        }
        
        Logger.LogInformation("Change dateTime {dateTime}. Old : {now}", dateTime, DateTime.UtcNow);

        ProcessStartInfo processStartInfo = new()
        {
            FileName = "/bin/date",
            Arguments = $"-s \"{dateTime:u}\"",
            UseShellExecute = false,
            RedirectStandardOutput = false,
            CreateNoWindow = true
        };

        using Process process = new();
        process.StartInfo = processStartInfo;

        try
        {
            process.Start();
            process.WaitForExit();
        }
        catch (Exception e)
        {
            Logger.LogWarning(e, "Change datetime in error");
        }
    }
    
    public bool IsShutdownAsked()
    {
        return WillSleep.HasValue;
    }
}