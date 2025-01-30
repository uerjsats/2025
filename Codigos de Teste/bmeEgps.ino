#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Criar instância do GPS e da Serial2
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // Objeto do sensor

void setup() {
    Serial.begin(115200);  // Comunicação com o PC
    gpsSerial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17

    Serial.println("Iniciando GPS e BME280...");

    if (!bme.begin(0x76)) { // Endereço padrão I2C do BME280
        Serial.println("Erro ao detectar o BME280!");
        while (1);  // Fica em loop infinito se o BME280 não for encontrado
    }
}

void loop() {
    // Lê os dados do GPS
    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    // Verifica se a localização do GPS foi atualizada
    if (gps.location.isUpdated()) {
        Serial.print("Latitude: "); 
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
    }

    // Exibe o horário UTC do GPS
    if (gps.time.isUpdated()) {
        Serial.printf("Horário (UTC): %02d:%02d:%02d\n", gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    // Lê os dados do BME280
    Serial.print("Temperatura: ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");
    
    Serial.print("Pressão: ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
    
    Serial.print("Altitude estimada: ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    
    // Umidade
    Serial.print("Umidade: ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.println();
    delay(2000); // Aguarda 2 segundos antes da próxima leitura
}
