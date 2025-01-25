#!/bin/bash

#set -e # Stop le programme s'il y a une erreur
set -x # Affiche les commandes

SERIAL_PORT="/dev/ttyS1"
MESHTASTIC_SERIAL_PORT="/dev/ttyS2"
DATA_OUTPUT_DIR="/mnt/sdcard/data"
REMOTE_DIR="valentin@192.168.1.254:/home/valentin/Data/cameras/opi"
SLEEP_DURATION=150

# Fonction pour setup la carte
startup() {
    echo "Setup de la carte"
    
    mkdir -p $DATA_OUTPUT_DIR
    
    # cpu
    cpufreq-set --governor conservative
    
    # configure UART
    stty -F $SERIAL_PORT 115200
    stty -F $MESHTASTIC_SERIAL_PORT 115200

    # configure leds
    echo cpu0 > /sys/class/leds/beaglebone\:green\:usr0/trigger
    echo none > /sys/class/leds/beaglebone\:green\:usr1/trigger
    echo none > /sys/class/leds/beaglebone\:green\:usr2/trigger
    echo none > /sys/class/leds/beaglebone\:green\:usr3/trigger
    
    sqlite3 "$DATA_OUTPUT_DIR/data.db" <<EOF
CREATE TABLE IF NOT EXISTS telemetry (
    uptime INTEGER,
    voltageBattery INTEGER,
    currentBattery INTEGER,
    voltageSolar INTEGER,
    currentSolar INTEGER,
    temperatureRtc REAL,
    temperatureBattery REAL,
    temperature REAL,
    humidity REAL,
    pressure REAL,
    createdAt TEXT
);

CREATE TABLE IF NOT EXISTS system (
    cpu INTEGER,
    ram INTEGER,
    disk INTEGER,
    sdcard INTEGER,
    uptime INTEGER,
    createdAt TEXT
);

CREATE TABLE IF NOT EXISTS meshtastic (
    id INTEGER,
    longName TEXT,
    shortName TEXT,
    snr REAL,
    lastHeard INTEGER,
    createdAt TEXT
);

CREATE TABLE IF NOT EXISTS aprs (
    callsign TEXT,
    content TEXT,
    snr REAL,
    rssi REAL,
    lastHeard INTEGER,
    createdAt TEXT
);
EOF
}

read_json_from_serial() {
    echo "Envoi de la commande sur le port série..."
    echo -e "json" > "$SERIAL_PORT"
    JSON_RESPONSE=$(timeout 5s cat "$SERIAL_PORT")

    if [ -n "$JSON_RESPONSE" ] && echo "$JSON_RESPONSE" | jq -r '.' > /dev/null 2>&1; then
        echo "JSON reçu: $JSON_RESPONSE"

        echo $"$JSON_RESPONSE" > "$DATA_OUTPUT_DIR/mcu.json"
        return 0
    else
        echo "Erreur : Réponse non valide"
        return 1
    fi
}

set_system_time() {
    TIMESTAMP_JSON=$(echo "$JSON_RESPONSE" | jq '.time')
    echo "Réglage de l'heure du système avec le timestamp : $TIMESTAMP_JSON"
    sudo date -s "@$TIMESTAMP_JSON"
    sudo hwclock -w -f /dev/rtc0
    TIMESTAMP=$(date +"%Y-%m-%d-%H-%M-%S")
    date
}

save_telemetries_and_aprs_to_database() {
    VALUES=$(echo "$JSON_RESPONSE" | jq -r '[.uptime, .energy.voltageBattery, .energy.currentBattery, .energy.voltageSolar, .energy.currentSolar, .box.temperatureRtc, .box.temperatureBattery, .weather.temperature, .weather.humidity, .weather.pressure, (now | todateiso8601)] | @csv')

    save_to_database telemetry

    VALUES=$(echo "$JSON_RESPONSE" | jq -r '.aprsReceived[] | [.callsign, .packet, .snr, .rssi, .time, (now | todateiso8601)] | @csv')

    save_to_database aprs
}

capture_photos() {
    echo "Capture des photos avec fswebcam pour chaque caméra..."
    for CAMERA_CONFIG in $DATA_OUTPUT_DIR/../scripts/camera_configs/*.conf; do
        CAMERA_NAME=$(basename "$CAMERA_CONFIG" .conf)
        
        if [ -e "/dev/v4l/by-id/$CAMERA_NAME" ]; then
            mkdir -p "$DATA_OUTPUT_DIR/camera/${CAMERA_NAME}"
            PHOTO_PATH="$DATA_OUTPUT_DIR/camera/${CAMERA_NAME}/${TIMESTAMP}.jpg"
            fswebcam -c "$CAMERA_CONFIG" --save "$PHOTO_PATH"
            echo "Photo capturée pour la caméra $CAMERA_NAME : $PHOTO_PATH"
        else
            echo "La camera $CAMERA_NAME est introuvable"
        fi
    done
}

generate_telemetry_image() {
    DATE=$(date +"%Y-%m-%d %H:%M:%S")
    BATTERY_VOLTAGE=$(echo "$JSON_RESPONSE" | jq '.energy.voltageBattery')
    BATTERY_CURRENT=$(echo "$JSON_RESPONSE" | jq '.energy.currentBattery')
    PANEL_VOLTAGE=$(echo "$JSON_RESPONSE" | jq '.energy.voltageSolar')
    PANEL_CURRENT=$(echo "$JSON_RESPONSE" | jq '.energy.currentSolar')
    TEMPERATURE=$(echo "$JSON_RESPONSE" | jq '.weather.temperature')
    HUMIDITY=$(echo "$JSON_RESPONSE" | jq '.weather.humidity')
    PRESSURE=$(echo "$JSON_RESPONSE" | jq '.weather.pressure')

    mkdir -p "$DATA_OUTPUT_DIR/telemetry"
    OUTPUT_FILE="$DATA_OUTPUT_DIR/telemetry/${TIMESTAMP}.jpg"

     #-fill lightgray -stroke lightgray -draw "rectangle 85,60 615,85" \
     #-fill indianred -stroke indianred -draw "rectangle 85,60 85,85" \
     #-font "Arial.ttf" -pointsize 20 -stroke black -strokewidth 1 -fill black \
         
    convert -size 700x480 xc:black \
        -font "Arial.ttf" -pointsize 30 -stroke white -strokewidth 1 -fill white \
        -annotate +185+30 "$DATE" \
        -font "Arial.ttf" -pointsize 20 -stroke white -strokewidth 1 -fill white \
        -annotate +135+80 "Tension batterie : ${BATTERY_VOLTAGE} mV" \
        -annotate +15+80 "11000" -annotate +620+80 "13500" \
        -annotate +135+130 "Courant batterie : ${BATTERY_CURRENT} mA" \
        -annotate +15+130 "0" -annotate +620+130 "1000" \
        -annotate +135+180 "Tension panneau : ${PANEL_VOLTAGE} mV" \
        -annotate +15+180 "0" -annotate +620+180 "25000" \
        -annotate +135+230 "Courant panneau : ${PANEL_CURRENT} mA" \
        -annotate +15+230 "0" -annotate +620+230 "4000" \
        -annotate +135+280 "Température : ${TEMPERATURE} °C" \
        -annotate +15+280 "-10" -annotate +620+280 "50" \
        -annotate +135+330 "Humidité : ${HUMIDITY} %" \
        -annotate +15+330 "0" -annotate +620+330 "100" \
        -annotate +135+380 "Pression : ${PRESSURE} hPa" \
        -annotate +15+380 "950" -annotate +620+380 "1050" \
        -quality 40 -format jpeg "$OUTPUT_FILE"

    echo "Image de télémétrie générée : $OUTPUT_FILE"
}

check_ethernet_connection() {
    if ip link | grep "state UP"; then
        return 0  # Connecté
    else
        return 1  # Non connecté
    fi
}

check_link_connection() {
    if ip addr | grep "192.168.1." || ip addr | grep "44.151.38."; then
        return 0  # Connecté
    else
        return 1  # Non connecté
    fi
}

sync_photos() {
    echo "Synchronisation des photos vers le dossier distant..."
    rsync -avz "$DATA_OUTPUT_DIR/" "$REMOTE_DIR"
}

save_system_info() {
    cpu_percentage=$(top -bn1 | grep "%Cpu(s)" | awk '{printf "%.0f", $2}')

    ram_percentage=$(free | grep Mem | awk '{printf "%.0f", (1 - $7/$2) * 100}')

    disk_percentage=$(df -h / | awk 'NR==2 {print $5}' | cut -d'%' -f1)
    sdcard_percentage=$(df -h /mnt/sdcard | awk 'NR==2 {print $5}' | cut -d'%' -f1)

    uptime_seconds=$(awk '{printf "%.0f", $1}' /proc/uptime)

    json="{ \"cpu\": $cpu_percentage, \"ram\": $ram_percentage, \"disk\": $disk_percentage, \"sdcard\": $sdcard_percentage, \"uptime\": $uptime_seconds }"

    echo $"$json" > "$DATA_OUTPUT_DIR/system.json"
    
    VALUES=$(echo "$json" | jq -r '[.cpu, .ram, .disk, .sdcard, .uptime, (now | todateiso8601)] | @csv')

    save_to_database system
}

save_meshtastic_nodes() {
    json=$(meshtastic --port $MESHTASTIC_SERIAL_PORT --info | sed -n '/Nodes in mesh:/,/Preferences:/p' | head -n -2 | sed '1s/.*/{/' | jq 'to_entries[] | {id: .value.user.id, longName: .value.user.longName, shortName: .value.user.shortName, snr: .value.snr, lastHeard: .value.lastHeard, hopsAway: .value.hopsAway''}' | jq -r -s '.')
    
    echo $"$json" > "$DATA_OUTPUT_DIR/meshtastic.json"
     
    VALUES=$(echo "$json" | jq -r '.[] | [.id, .longName, .shortName, .snr, .lastHeard, (now | todateiso8601)] | @csv')

    save_to_database meshtastic
}

save_to_database() {
    echo $"$VALUES" > /tmp/data.csv

    sqlite3 "$DATA_OUTPUT_DIR/data.db" <<EOF
.mode csv
.import /tmp/data.csv $1
EOF

    rm /tmp/data.csv
}

startup

while true; do
    TIMESTAMP=$(date +"%Y-%m-%d-%H-%M-%S")
    date

#    if read_json_from_serial; then    
#        set_system_time
#        save_telemetries_and_aprs_to_database
        #generate_telemetry_image
#    fi
    
#    capture_photos
#    save_system_info
    save_meshtastic_nodes
    exit 1

    if check_ethernet_connection; then
        if check_link_connection; then
            echo "Liaison OK. On synchronise les fichiers"
 #           sync_photos
        fi
        date
        sleep $SLEEP_DURATION
    else
        echo "Ethernet non connecté. On attend 10 secondes avant de dormir $SLEEP_DURATION secondes..."
        sleep 10
        date
        rtcwake -d /dev/rtc0 -m mem -s "$SLEEP_DURATION"
    fi
done

