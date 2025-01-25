using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Monitor.Migrations
{
    /// <inheritdoc />
    public partial class Meshtastic : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<bool>(
                name: "IsMeshtastic",
                table: "LoRas",
                type: "INTEGER",
                nullable: false,
                defaultValue: false);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "IsMeshtastic",
                table: "LoRas");
        }
    }
}
