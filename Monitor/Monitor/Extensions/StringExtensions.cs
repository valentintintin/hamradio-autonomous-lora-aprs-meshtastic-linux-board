using System.Globalization;
using AprsSharp.AprsParser;

namespace Monitor.Extensions;

public static class StringExtensions
{
    public static Packet ToAprsPacket(this string value)
    {
        try
        {
            return new Packet(value);
        }
        catch (Exception)
        {
            return new Packet($"{value.Split(":")[0]}: ");
        }
    }
    
    public static uint ToInteger(this string hexString)
    {
        return uint.Parse(hexString.TrimStart('!'), NumberStyles.HexNumber);
    }
}