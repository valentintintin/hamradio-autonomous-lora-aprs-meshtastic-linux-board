namespace Monitor.Extensions;

public static class IntExtensions
{
    public static string ToHexString(this uint integer)
    {
        return $"!{integer.ToString("X").ToLower()}";
    }
}