using System.Text.Json.Serialization;

namespace Monitor.Models.SerialMessages;

public class GpioData : Message
{
    // [JsonPropertyName("relay1")]
    public bool Wifi { get; set; }
    
    [JsonPropertyName("relay2")]
    public bool Npr { get; set; }
    
    [JsonPropertyName("relay1")]
    public bool Meshtastic { get; set; }
    
    [JsonPropertyName("ldr")]
    public int Ldr { get; set; }
}