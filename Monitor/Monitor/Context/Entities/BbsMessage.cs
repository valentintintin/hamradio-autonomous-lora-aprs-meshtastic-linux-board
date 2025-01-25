using System.ComponentModel.DataAnnotations;

namespace Monitor.Context.Entities;

public class BbsMessage : IEntity
{
    public long Id { get; set; }
    public DateTime CreatedAt { get; set; }
    
    [MaxLength(16)]
    public required string From { get; set; }
    
    [MaxLength(16)]
    public required string To { get; set; }
    
    [MaxLength(200)]
    public required string Message { get; set; }
    
    public DateTime? RemindedAt { get; set; }
    
    public DateTime? ReadAt { get; set; }
}