using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using Monitor.Components;
using Monitor.Exceptions;
using Monitor.Extensions;
using Monitor.Models;

namespace Monitor.Services;

public class CameraService : AService
{
    private bool _isRunning;

    private readonly string _storagePath;
    private readonly string? _message;
    private readonly List<FswebcamParameters> _fswebcamParameters;
    private readonly int _frameToTakeDuringNight;

    private readonly string _fontPath = Path.Combine(Path.GetDirectoryName(Assembly.GetEntryAssembly()!.Location)!, "Arial.ttf");

    private const int WidthData = 700;
    private const int HeightData = 480;
    private const int LegendSize = 70;
    private const int LegendMargin = 15;
    private const int HeightProgressBar = 25;
    private readonly int _widthMaxProgressBar;

    public CameraService(ILogger<CameraService> logger, IConfiguration configuration) : base(logger)
    {
        var configurationSection = configuration.GetSection("Cameras");
        _message = configurationSection.GetValue<string?>("Message");
        _fswebcamParameters = configurationSection.GetSection("Devices").Get<List<FswebcamParameters>>()?.ToList() ?? [];
        _frameToTakeDuringNight = configurationSection.GetValue("FrameToTakeDuringNight", 1);
        
        _storagePath = Path.Combine(
            configuration.GetValueOrThrow<string>("StoragePath"), 
            configurationSection.GetValueOrThrow<string>("Path")
        );
        Directory.CreateDirectory($"{_storagePath}");
        
        _widthMaxProgressBar = WidthData - (LegendSize + LegendMargin) * 2;
    }

    public string? GetFinalLast()
    {
        var path = $"{_storagePath}/last.jpg";

        if (!File.Exists(path))
        {
            Logger.LogWarning("Last final image does not exist at {path}", path);
            
            return null;
        }
        
        var resolveLinkTarget = File.ResolveLinkTarget(path, true);

        if (resolveLinkTarget == null)
        {
            return path;
        }

        return resolveLinkTarget?.Exists == true ? resolveLinkTarget.FullName : null;
    }

    public List<(string path, FswebcamParameters parameters)> MakeAllImages(string? directory = null, string? fileName = null)
    {
        Logger.LogInformation("Make all images");
        
        if (_isRunning)
        {
            throw new WarningException("Already running");
        }

        _isRunning = true;
        
        var now = DateTime.UtcNow;
        directory ??= $"{_storagePath}/{now:yyyy-MM-dd}";
        Directory.CreateDirectory(directory);
        fileName ??= $"{now:yyyy-MM-dd-HH-mm-ss}.jpg";
        
        var images = CaptureAllCameras(directory, fileName);
        
        Directory.CreateDirectory($"{directory}/data");
        var imageDataOutputFile = $"{directory}/data/{fileName}";
        CreateDataImage(imageDataOutputFile);
        images.Add((imageDataOutputFile, new FswebcamParameters
        {
            Device = "data",
            SaveFile = imageDataOutputFile,
            Resolution = $"{WidthData}x{HeightData}",
        }));

        _isRunning = false;

        return images;
    }

    public async Task<MemoryStream> CaptureAndCreateMergedImage(bool save = true)
    {
        Logger.LogInformation("Create final image");

        try
        {
            var now = DateTime.UtcNow;
            var directory = $"{_storagePath}/{now:yyyy-MM-dd}";
            var fileName = $"{now:yyyy-MM-dd-HH-mm-ss}.jpg";
            Directory.CreateDirectory(directory);
            
            var images = MakeAllImages(directory, fileName);

            Logger.LogDebug("We have {count} photos", images.Count);
            
            if (images.Count == 0)
            {
                throw new Exception("No image to merge");
            }
            
            var width = images.Sum(i => i.parameters.Width);
            var height = images.Max(i => i.parameters.Height);
            var imageMagickParameters = new ImageMagickParameters();

            imageMagickParameters.CreateCanvas(width, height, "black");

            foreach (var image in images)
            {
                imageMagickParameters.OverlayImage(image.path, 0, 0).Translate(image.parameters.Width, 0);
            }

            var tempOutputFile = $"{_storagePath}/output.jpg";
            imageMagickParameters.SetQuality(80).SetFormat("jpeg").Output(tempOutputFile);
            ImageMagick(imageMagickParameters);

            var lastPath = $"{_storagePath}/last.jpg";
            File.Delete(lastPath);
            File.CreateSymbolicLink(lastPath, tempOutputFile);
            
            MemoryStream stream = new();

            await using (FileStream fileStream = new(tempOutputFile, FileMode.OpenOrCreate, FileAccess.Read))
            {
                await fileStream.CopyToAsync(stream);
                stream.Seek(0, SeekOrigin.Begin);
            }

            Logger.LogDebug("Saving image to stream. Size : {size}", stream.Length);

            if (save)
            {
                foreach (var (path, parameters) in images)
                {
                    var deviceName = parameters.NameSanitized;
                    
                    Directory.CreateDirectory($"{directory}/{deviceName}");

                    var dest = $"{directory}/{deviceName}/{fileName}";
                    if (File.Exists(dest))
                    {
                        Logger.LogWarning("File {file} already exist", dest);
                        continue;
                    }
                    
                    File.Copy(path, dest);
                }

                var filePath = $"{directory}/{fileName}";

                Logger.LogInformation("Save final image to {path}", filePath);

                await using var fileStream = File.Create(filePath);
                await stream.CopyToAsync(fileStream);
                stream.Seek(0, SeekOrigin.Begin);
            }

            Logger.LogInformation("Create final image OK");
            
            return stream;
        }
        finally
        {
            _isRunning = false;
        }
    }

    private List<(string path, FswebcamParameters parameters)> CaptureAllCameras(string directory, string fileName)
    {
        Logger.LogDebug("Will capture images from cameras : {cameras}", _fswebcamParameters.Select(j => j.Device).JoinString());

        return _fswebcamParameters.Select(parameters =>
        {
            var cameraName = parameters.NameSanitized;

            Logger.LogTrace("Capture image from {camera}", cameraName);

            Directory.CreateDirectory($"{directory}/{cameraName}");

            parameters.Frames = MonitorService.State.Mppt.Night ? _frameToTakeDuringNight : null;
            parameters.SaveFile = $"{directory}/{cameraName}/{fileName}";
            
            return (CaptureImage(parameters), parameters);
        }).ToList();
    }

    private void CreateDataImage(string filePath)
    {
        var now = DateTime.UtcNow;
        var imageMagickParameters = new ImageMagickParameters();
        
        imageMagickParameters.CreateCanvas(WidthData, HeightData, "black");
        imageMagickParameters.AddText(30, _fontPath, LegendMargin + LegendSize + 100, 30, $"{now.ToFrench():G}", "white");

        DrawProgressBarwithInfo(imageMagickParameters, 0, "Tension batterie", "mV", EntitiesManagerService.Entities.BatteryVoltage.Value, 11000, 13500, "indianred");
        DrawProgressBarwithInfo(imageMagickParameters, 1, "Courant batterie", "mA", EntitiesManagerService.Entities.BatteryCurrent.Value, 0, 1000, "cyan");
        DrawProgressBarwithInfo(imageMagickParameters, 2, "Tension panneau", "mV", EntitiesManagerService.Entities.SolarVoltage.Value, 0, 25000, "yellow");
        DrawProgressBarwithInfo(imageMagickParameters, 3, "Courant panneau", "mA", EntitiesManagerService.Entities.SolarCurrent.Value, 0, 4000, "lightgreen");
        DrawProgressBarwithInfo(imageMagickParameters, 4, "Température", "°C", Math.Round(EntitiesManagerService.Entities.WeatherTemperature.Value, 2), -10, 50, "darkorange");
        DrawProgressBarwithInfo(imageMagickParameters, 5, "Humidité", "%", Math.Round(EntitiesManagerService.Entities.WeatherHumidity.Value, 0), 0, 100, "lightblue");
        DrawProgressBarwithInfo(imageMagickParameters, 6, "Pression", "hPa", Math.Round(EntitiesManagerService.Entities.WeatherPressure.Value, 0), 950, 1050, "lightpink");
            
        if (!string.IsNullOrWhiteSpace(_message))
        {
            imageMagickParameters.AddText(15, _fontPath, WidthData - 15 * _message.Length, HeightData - 30, _message, "white");
        }

        imageMagickParameters.SetQuality(40).SetFormat("jpeg").Output(filePath);
        ImageMagick(imageMagickParameters);
    }
    
    private void DrawProgressBarwithInfo(ImageMagickParameters parameters, int indexValue, string label, string unit, double value, double min, double max, string colorBar)
    {
        var y = indexValue * 50 + 80;
        var valueBar = value;
        
        if (valueBar > max)
        {
            valueBar = max;
        } 
        else if (value < min)
        {
            valueBar = min;
        }

        valueBar = (int)Math.Round((double)Utils.MapValue(valueBar, min, max, 0, _widthMaxProgressBar), 2);
        
        parameters
            .DrawRectangle(LegendMargin + LegendSize, y + 5, _widthMaxProgressBar, HeightProgressBar, "lightgray")
            .DrawRectangle(LegendMargin + LegendSize, y + 5, (int) valueBar, HeightProgressBar, colorBar)
            .AddText(20, _fontPath, LegendMargin + LegendSize + 50, y, $"{label} : {value} {unit}", "black")
            .AddText(20, _fontPath, LegendMargin, y, $"{min.ToString(CultureInfo.CurrentCulture)}", colorBar)
            .AddText(20, _fontPath, LegendMargin + LegendSize + _widthMaxProgressBar + 5, y, $"{max.ToString(CultureInfo.CurrentCulture)}", colorBar)
            ;
    }

    private string CaptureImage(FswebcamParameters parameters)
    {
        Process process = new()
        {
            StartInfo = new ProcessStartInfo
            {
                FileName = "fswebcam",
                Arguments = parameters.ToString(),
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true
            }
        };
        
        Logger.LogTrace("Start process: fswebcam for {video}", parameters.Device);
        Logger.LogDebug("fswebcam {parameter}", parameters.ToString());

        process.Start();
        if (!process.WaitForExit(TimeSpan.FromMinutes(2)))
        {
            process.Kill();
            throw new WebcamException($"Timeout for fswebcam {parameters}");
        }

        var errorStream = process.StandardError.ReadToEnd();
        var standardStream = process.StandardOutput.ReadToEnd();
        var stream = standardStream + " " + errorStream;
        
        Logger.LogDebug("Image capture output for {device} : {stream}", parameters.Device, stream);
        
        if (process.ExitCode == 0)
        {
            if (stream.Contains("No frames captured"))
            {
                throw new WebcamException(stream);
            }
            
            Logger.LogInformation("Image captured successfully for device {device}", parameters.Device);

            return parameters.SaveFile;
        }

        Logger.LogError("Error capturing image for device {device}. Exit code: {exitCode}", parameters.Device, process.ExitCode);
            
        throw new WebcamException(stream);
    }
    
    private void ImageMagick(ImageMagickParameters parameters)
    {
        Process process = new()
        {
            StartInfo = new ProcessStartInfo
            {
                FileName = "convert",
                Arguments = parameters.ToString(),
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                CreateNoWindow = true
            }
        };
        
        Logger.LogTrace("Start process: convert");
        Logger.LogDebug("convert {command}", parameters.ToString());

        process.Start();
        if (!process.WaitForExit(TimeSpan.FromMinutes(2)))
        {
            process.Kill();
            throw new WebcamException($"Timeout for convert {parameters}");
        }
        
        var errorStream = process.StandardError.ReadToEnd();
        var standardStream = process.StandardOutput.ReadToEnd();
        var streamString = standardStream + " " + errorStream;
        
        if (!string.IsNullOrWhiteSpace(errorStream))
        {
            Logger.LogError("Convert error with exit code {ProcessExitCode}. Command: {convertParametersters}. {OutputStreamStringamString}", process.ExitCode, parameters, streamString);
            
            throw new WebcamException($"ImageMagick error with exit code {process.ExitCode}.\n\nCommand:\n\nconvert {parameters}.\n\nOutput:\n\n{streamString}");
        }
    }
}