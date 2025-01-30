#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Criar instância do GPS e da Serial2
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

void setup() {
    Serial.begin(115200);  // Comunicação com o PC
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

    Serial.println("Iniciando GPS...");
}

void loop() {
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    if (gps.location.isUpdated()) {
        Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
    }

    if (gps.time.isUpdated()) {
        Serial.printf("Horário (UTC): %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    }
}
