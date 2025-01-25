using System.Text.Json.Serialization;

namespace Monitor.Models.SerialMessages;

public class McuSystemData : Message
{
    [JsonPropertyName("state")]
    public required string State { get; set; }

    [JsonPropertyName("boxOpened")]
    public bool BoxOpened { get; set; }
    
    [JsonPropertyName("time")]
    public long Timestamp { get; set; }

    [JsonPropertyName("uptime")] 
    public long Uptime { get; set; }

    [JsonPropertyName("nbError")] 
    public int NbError { get; set; }

    [JsonPropertyName("mpptWatchdogSafetyTimer")]
    public long MpptWatchdogSafetyTimer { get; set; }

    [JsonPropertyName("mpptWatchdogSafety")]
    public bool MpptWatchdogSafetyEnabled { get; set; }

    [JsonPropertyName("aprsDigipeater")]
    public bool AprsDigipeaterEnabled { get; set; }

    [JsonPropertyName("aprsTelemetry")]
    public bool AprsTelemetryEnabled { get; set; }

    [JsonPropertyName("aprsPosition")]
    public bool AprsPositionEnabled { get; set; }

    [JsonPropertyName("sleep")]
    public bool Sleep { get; set; }

    [JsonPropertyName("resetError")]
    public bool ResetError { get; set; }

    [JsonPropertyName("watchdog")]
    public bool Watchdog { get; set; }

    [JsonPropertyName("watchdogLoraTx")]
    public bool WatchdogLoraTx { get; set; }

    [JsonPropertyName("watchdogLoraTxTimer")]
    public long WatchdogLoraTxTimer { get; set; }

    [JsonPropertyName("temperatureRtc")]
    public float TemperatureRtc { get; set; }

    [JsonIgnore]
    public bool IsAlert => State == "alert";

    [JsonIgnore]
    public DateTimeOffset DateTime => DateTimeOffset.FromUnixTimeSeconds(Timestamp);
    [JsonIgnore]
    public TimeSpan UptimeTimeSpan => TimeSpan.FromSeconds(Uptime);
}