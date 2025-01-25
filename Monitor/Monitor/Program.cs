using System.Globalization;
using System.Text.Json.Serialization;
using AntDesign;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.FileProviders;
using Monitor;
using Monitor.Components;
using Monitor.Context;
using Monitor.Extensions;
using Monitor.Services;
using Monitor.Workers;
using Monitor.Workers.Enableables;
using NetDaemon.Extensions.Scheduler;
using NLog;
using NLog.Web;
using LogLevel = Microsoft.Extensions.Logging.LogLevel;

Console.WriteLine("Starting");
// GetSunCalc(); return;

var logger = ConfigureNlogFile();

try
{
    var builder = WebApplication.CreateBuilder(args);

    builder.Logging.ClearProviders().SetMinimumLevel(LogLevel.Trace);
    builder.Host.UseNLog();
    
    builder.Services.AddRazorComponents()
        .AddInteractiveServerComponents();
    builder.Services.AddAntDesign();
    builder.Services.AddControllers().AddJsonOptions(options =>
    {
        options.JsonSerializerOptions.Converters.Add(new JsonStringEnumConverter());
    });

    builder.Services.AddResponseCompression();

    builder.Services.AddNetDaemonScheduler();

    builder.Services.Configure<HostOptions>(hostOptions =>
    {
        hostOptions.BackgroundServiceExceptionBehavior = BackgroundServiceExceptionBehavior.Ignore;
    });

    builder.Services.AddDbContextFactory<DataContext>(option =>
    {
        option.UseSqlite(
            builder.Configuration.GetConnectionString("Default")!
                .Replace("{StoragePath}", builder.Configuration.GetValueOrThrow<string>("StoragePath"))
        );
    });
    
    builder.Services.AddSingleton<EntitiesManagerService>();
    builder.Services.AddSingleton<MonitorService>();
    builder.Services.AddSingleton<SerialMessageService>();
    builder.Services.AddSingleton<SystemService>();
    builder.Services.AddSingleton<CameraService>();

    builder.Services.AddSingleton<MpptApp>();
    builder.Services.AddSingleton<WatchdogApp>();
    builder.Services.AddSingleton<BatteryApp>();
    builder.Services.AddSingleton<GpioApp>();
    builder.Services.AddSingleton<CameraCaptureApp>();
    builder.Services.AddSingleton<SerialPortMcuCommandsApp>();
    builder.Services.AddSingleton<AprsIsApp>();
    builder.Services.AddSingleton<McuFeatureApp>();
    builder.Services.AddSingleton<MeshtasticApp>();

    builder.Services.AddHostedService(services => services.GetRequiredService<MpptApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<GpioApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<SerialPortMcuCommandsApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<WatchdogApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<BatteryApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<CameraCaptureApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<AprsIsApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<McuFeatureApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<MeshtasticApp>());
    builder.Services.AddHostedService(services => services.GetRequiredService<EntitiesManagerService>());

    var app = builder.Build();

    if (!builder.Environment.IsDevelopment())
    {
        app.UseExceptionHandler("/error", createScopeForErrors: true);
    }

    app.UseMiddleware<PerformanceAndCultureMiddleware>();
    app.UseResponseCompression();

    app.UseStaticFiles();

    app.UseRouting();

    app.UseAntiforgery();

    app.MapControllers();

    app.MapRazorComponents<App>()
        .AddInteractiveServerRenderMode();

    var storagePath = app.Configuration.GetValueOrThrow<string>("StoragePath");

    Directory.CreateDirectory(storagePath);

    app.UseStaticFiles(new StaticFileOptions
    {
        FileProvider = new PhysicalFileProvider(storagePath),
        RequestPath = "/storage"
    });

    using IServiceScope scope = app.Services.GetService<IServiceScopeFactory>()!.CreateAsyncScope();
    await scope.ServiceProvider.GetRequiredService<DataContext>().Database.MigrateAsync();

    LocaleProvider.DefaultLanguage = "fr-FR";
    LocaleProvider.SetLocale(LocaleProvider.DefaultLanguage);

    var culture = CultureInfo.GetCultureInfo(LocaleProvider.DefaultLanguage);

    CultureInfo.DefaultThreadCurrentCulture = culture;
    CultureInfo.DefaultThreadCurrentUICulture = culture;
    CultureInfo.CurrentCulture = culture;
    Thread.CurrentThread.CurrentCulture = culture;
    Thread.CurrentThread.CurrentUICulture = culture;
    
    Console.WriteLine("Started");

    await app.RunAsync();
}
catch (Exception exception)
{
    // For dotnet ef commands
    var type = exception.GetType().Name;
    var isMigrationCommand = type.StartsWith("HostAbortedException");

    if (!isMigrationCommand)
    {
        // NLog: catch setup errors
        logger.Error(exception, "Stopped program because of exception");
        throw;
    }
}
finally
{
    // Ensure to flush and stop internal timers/threads before application-exit (Avoid segmentation fault on Linux)
    LogManager.Shutdown();
}

Console.WriteLine("Stopped");

// void GetSunCalc()
// {
//     var configurationSection = app.Configuration.GetSection("Position");
//
//     (double latitude, double longitude, int altitude) position = (
//         configurationSection.GetValueOrThrow<double>("Latitude"),
//         configurationSection.GetValueOrThrow<double>("Longitude"),
//         configurationSection.GetValueOrThrow<int>("Altitude")
//     );
//
//     SunCalc.GetSunPhases(DateTime.UtcNow, position.latitude, position.longitude, position.altitude)
//         .Concat(SunCalc.GetSunPhases(DateTime.UtcNow.AddDays(1), position.latitude, position.longitude, position.altitude))
//         .OrderBy(a => a.PhaseTime)
//         .ForEach(
//     a =>
//     {
//         Console.WriteLine(a.Name + "  " + a.PhaseTime.ToFrench());
//     });
// }

Logger ConfigureNlogFile()
{
    var environment = Environment.GetEnvironmentVariable("ASPNETCORE_ENVIRONMENT");
    var configFile = "nlog.config";

    if (File.Exists($"nlog.{environment}.config"))
    {
        configFile = $"nlog.{environment}.config";
    }

    if (string.IsNullOrWhiteSpace(environment))
    { // Null == Production
        configFile = "nlog.Production.config";
    }

    Console.WriteLine($"Logger avec le fichier {configFile}");
    
    return LogManager.Setup().LoadConfigurationFromFile(configFile).GetCurrentClassLogger();
}