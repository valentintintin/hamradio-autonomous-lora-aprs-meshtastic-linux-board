using Microsoft.EntityFrameworkCore.Migrations;

#nullable disable

namespace Monitor.Migrations
{
    /// <inheritdoc />
    public partial class NbError : Migration
    {
        /// <inheritdoc />
        protected override void Up(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.AddColumn<int>(
                name: "NbError",
                table: "Systems",
                type: "INTEGER",
                nullable: false,
                defaultValue: 0);
        }

        /// <inheritdoc />
        protected override void Down(MigrationBuilder migrationBuilder)
        {
            migrationBuilder.DropColumn(
                name: "NbError",
                table: "Systems");
        }
    }
}
