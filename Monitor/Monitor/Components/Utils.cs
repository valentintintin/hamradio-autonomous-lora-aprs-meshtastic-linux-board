using AntDesign.Charts;
using Monitor.Models;

namespace Monitor.Components;

public static class Utils
{
    public static int MapValue(double value, double inMin, double inMax, double outMin, double outMax)
    {
        return (int)Math.Round((value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin);
    }
    
    public static LineConfig GetLineConfig(string yAxisText, double? min = null, double? max = null)
    {
        return new LineConfig
        {
            Padding = "auto",
            AutoFit = true,
            XField = nameof(LineChartData<object>.date),
            YField = nameof(LineChartData<object>.value),
            YAxis = new ValueAxis
            {
                Label = new BaseAxisLabel
                {
                    Visible = true
                },
                Title = new BaseAxisTitle
                {
                    Text = yAxisText,
                    Visible = true
                },
                Min = min,
                Max = max,
                Visible = true,
            },
            Legend = new Legend
            {
                Visible = false
            },
            SeriesField = nameof(LineChartData<object>.type),
            Color = new [] { "blue", "red" }
        };
    }
}