using Meshtastic.Protobufs;
using Monitor.Extensions;

namespace Monitor.Models;

public class MeshtasticNode(NodeInfo nodeInfo)
{
    public string Name { get; init; } = $"{nodeInfo.User?.LongName} ({nodeInfo.User?.ShortName}) - {nodeInfo.User?.Id} ({nodeInfo.Num})";

    public DateTimeOffset LastHeard { get; init; } = DateTimeOffset.FromUnixTimeSeconds(nodeInfo.LastHeard);

    public TimeSpan LastHeardFromNow => DateTime.UtcNow.ToFrench() - LastHeard;
}