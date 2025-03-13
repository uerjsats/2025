#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>

#define SEALEVELPRESSURE_HPA (1010)
#define DHTPIN 48        
#define DHTTYPE DHT22   
#define TIMEZONE_OFFSET -3  

const int eepromAddress = 0x50; // Substituir endereço pelo endereço obtido através do Scanner I2C

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_BME280 bme; 
DHT dht(DHTPIN, DHTTYPE); 
Adafruit_MPU6050 mpu;

void writeEEPROM(int address, const char* data, int length) {
  for (int i = 0; i < length; i++) {
    if (readEEPROMByte(address + i) != 0xFF) {
      Serial.println("EEPROM já possui dados, pulando escrita."); //Pula escrita se ja tiver dados naquele endereço
      return;
    }
  }

  Wire.beginTransmission(eepromAddress);
  Wire.write((address >> 8) & 0xFF); // Parte alta do endereço
  Wire.write(address & 0xFF);       // Parte baixa do endereço

  for (int i = 0; i < length; i++) {
    Wire.write(data[i]);
    if ((i + 1) % 64 == 0) {  
      Wire.endTransmission();
      delay(5); 
      Wire.beginTransmission(eepromAddress);
      Wire.write(((address + i + 1) >> 8) & 0xFF);
      Wire.write((address + i + 1) & 0xFF);
    }
  }

  Wire.endTransmission();
  delay(5); 
}

byte readEEPROMByte(int address) {
  Wire.beginTransmission(eepromAddress);
  Wire.write((address >> 8) & 0xFF); 
  Wire.write(address & 0xFF);       
  Wire.endTransmission();

  Wire.requestFrom(eepromAddress, 1); 
  return Wire.read(); 
}

void setup() {
  Serial.begin(115200);  
  gpsSerial.begin(9600, SERIAL_8N1, 45, 46);  

  bme.begin(0x76); 
  dht.begin(); 
  Wire.begin();
  mpu.begin(); 
}

void loop() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    gps.encode(c);
  }

  float tempBME = bme.readTemperature();
  float pressao = bme.readPressure() / 100.0F;
  float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  float umidadeBME = bme.readHumidity();

  float tempDHT = dht.readTemperature();
  float humDHT = dht.readHumidity();

  if (isnan(tempDHT) || isnan(humDHT)) {
    tempDHT = 0.0;
    humDHT = 0.0;
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Formatar os dados
  char dataBuffer[64];
  snprintf(dataBuffer, sizeof(dataBuffer), "%.2f:%d:%.2f:%.2f:%.6f:%.6f:%d:%.2f:%.2f:%.2f:8:9:89", 
           tempBME, gps.time.second(), pressao, altitude, 
           gps.location.lat(), gps.location.lng(), gps.satellites.value(), 
           a.acceleration.x, a.acceleration.y, a.acceleration.z);

  Serial.println(dataBuffer);

  // Gravar dados na EEPROM a partir do endereço 0, se estiver vazio
  writeEEPROM(0, dataBuffer, strlen(dataBuffer) + 1);

  delay(2000); 
}
