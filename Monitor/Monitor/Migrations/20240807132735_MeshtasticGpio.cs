using System;
using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Monitor.Migrations
{
    /// <inheritdoc />
    public partial class MeshtasticGpio : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<bool>(
                name: "Meshtastic",
                table: "Systems",
                type: "INTEGER",
                nullable: false,
                defaultValue: false);

            migrationBuilder.CreateTable(
                name: "BbsMessages",
                columns: table => new
                {
                    Id = table.Column<long>(type: "INTEGER", nullable: false)
                        .Annotation("Sqlite:Autoincrement", true),
                    CreatedAt = table.Column<DateTime>(type: "TEXT", nullable: false),
                    From = table.Column<string>(type: "TEXT", maxLength: 16, nullable: false),
                    To = table.Column<string>(type: "TEXT", maxLength: 16, nullable: false),
                    Message = table.Column<string>(type: "TEXT", maxLength: 200, nullable: false),
                    RemindedAt = table.Column<DateTime>(type: "TEXT", nullable: true),
                    ReadAt = table.Column<DateTime>(type: "TEXT", nullable: true)
                },
                constraints: table =>
                {
                    table.PrimaryKey("PK_BbsMessages", x => x.Id);
                });
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropTable(
                name: "BbsMessages");

            migrationBuilder.DropColumn(
                name: "Meshtastic",
                table: "Systems");
        }
    }
}
