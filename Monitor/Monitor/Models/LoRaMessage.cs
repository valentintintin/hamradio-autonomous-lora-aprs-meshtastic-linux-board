using Monitor.Context.Entities;

namespace Monitor.Models;

public class LoRaMessage(LoRa l)
{
    public string Type { get; init; } = l.IsMeshtastic ? "Meshtastic" : "APRS";

    public string? Sender { get; init; } = l.Sender;

    public string Frame { get; init; } = l.Frame;

    public DateTime CreatedAt { get; init; } = l.CreatedAt;

    public bool IsTx { get; init; } = l.IsTx;
}