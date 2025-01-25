FROM mcr.microsoft.com/dotnet/aspnet:8.0.6-bookworm-slim-arm32v7
RUN apt -y update && apt -y install fswebcam imagemagick
WORKDIR /App
#COPY . .
ENTRYPOINT ["dotnet", "Monitor.dll"]