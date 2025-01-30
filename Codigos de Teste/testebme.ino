#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // Objeto do sensor

void setup() {
    Serial.begin(115200);
    if (!bme.begin(0x76)) { // Endereço padrão I2C do BME280
        Serial.println("Erro ao detectar o BME280!");
        while (1);
    }
}

void loop() {
    Serial.print("Temperatura: ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");
    
    Serial.print("Pressão: ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");
    
    Serial.print("Altitude estimada: ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");
    
    Serial.print("Umidade: ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
    
    Serial.println();
    delay(2000); // Aguarda 2 segundos antes da próxima leitura
}
