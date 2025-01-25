namespace Monitor.Models;

public class LoRaStates
{
    public LimitedList<LoRaMessage> AprsRx { get; set; } = new(20);
    public LimitedList<LoRaMessage> AprsTx { get; set; } = new(20);
    
    public LimitedList<LoRaMessage> MeshtasticRx { get; set; } = new(20);
    public LimitedList<LoRaMessage> MeshtasticTx { get; set; } = new(20);

    public List<MeshtasticNode> MeshtasticNodes { get; set; } = [];
}