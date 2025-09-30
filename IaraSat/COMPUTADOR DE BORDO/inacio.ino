#include <Wire.h>
#include <Arduino.h>

#include <Adafruit_BME280.h>
#include <DHT.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ========== Definições ==========
#define SDA_PIN 42
#define SCL_PIN 41

#define DHT_PIN 48
#define DHT_TYPE DHT22 

#define PRESSAO_NIVEL_MAR 1013.25 // hPa

// ========== Objetos dos sensores ==========
Adafruit_BME280 bme;               // Sensor de pressão/temperatura
DHT dht(DHT_PIN, DHT_TYPE);        // Sensor de umidade/temperatura
Adafruit_MPU6050 mpu;              // Sensor de aceleração/giroscópio

// ========== Setup ==========
void setup() 
{
  Serial.begin(115200);
  while (!Serial); // Aguarda inicialização da porta serial
  Wire.begin(SDA_PIN, SCL_PIN);
    
  // ----- BME280 -----
  if (!bme.begin(0x76)) 
  {
    Serial.println("Erro ao inicializar o sensor BME280 no endereço 0x76!");
  }else
    {
      Serial.println("Sensor BME inicializado no endereço 0x76");
    }
  // ----- DHT22 -----
  dht.begin();

  // ----- MPU6050 -----
  if (!mpu.begin(0x68)) 
  {
    Serial.println("Erro ao inicializar o sensor MPU6050!");
  } else 
  {
    Serial.println("Sensor MPU6050 inicializado com sucesso.");
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    Serial.print("Faixa do giroscópio: ");
    switch (mpu.getGyroRange()) {
      case MPU6050_RANGE_250_DEG: Serial.println("±250 °/s"); break;
      case MPU6050_RANGE_500_DEG: Serial.println("±500 °/s"); break;
      case MPU6050_RANGE_1000_DEG: Serial.println("±1000 °/s"); break;
      case MPU6050_RANGE_2000_DEG: Serial.println("±2000 °/s"); break;
    }

    Serial.print("Filtro de banda definido para: ");
    switch (mpu.getFilterBandwidth()) {
      case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
      case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
      case MPU6050_BAND_94_HZ:  Serial.println("94 Hz"); break;
      case MPU6050_BAND_44_HZ:  Serial.println("44 Hz"); break;
      case MPU6050_BAND_21_HZ:  Serial.println("21 Hz"); break;
      case MPU6050_BAND_10_HZ:  Serial.println("10 Hz"); break;
      case MPU6050_BAND_5_HZ:   Serial.println("5 Hz"); break;
    }
  }

  Serial.println("\nLEITURA DOS DADOS DOS SENSORES:\n");
}

// ========== Loop ==========
void loop() 
{
  delay(2000); 

  // ----- Leitura BME280 -----
  float temp_bme = bme.readTemperature();
  float pressao_bme = bme.readPressure() / 100.0F;  // Conversão para hPa
  float altitude_bme = bme.readAltitude(PRESSAO_NIVEL_MAR);

  // ----- Leitura DHT22 -----
  float temp_dht_f = dht.readTemperature(true);  // true = Fahrenheit
  float umidade_dht = dht.readHumidity();

  if (isnan(temp_dht_f) || isnan(umidade_dht)) 
  {
    Serial.println("Erro ao ler dados do sensor DHT22.");
    return;
  }

  float sensacao_termica = dht.computeHeatIndex(temp_dht_f, umidade_dht, true); // true = Fahrenheit

  // ----- Leitura MPU6050 -----
  sensors_event_t aceleracao, giroscopio, temp_mpu;
  mpu.getEvent(&aceleracao, &giroscopio, &temp_mpu);

  // ========== Impressão dos dados ==========
  Serial.println("===== SENSOR BME280 =====");
  Serial.print("Temperatura: "); Serial.print(temp_bme); Serial.println(" °C");
  Serial.print("Pressão: "); Serial.print(pressao_bme); Serial.println(" hPa");
  Serial.print("Altitude: "); Serial.print(altitude_bme); Serial.println(" m\n");

  Serial.println("===== SENSOR DHT22 =====");
  Serial.print("Temperatura: "); Serial.print(temp_dht_f); Serial.println(" °F");
  Serial.print("Umidade: "); Serial.print(umidade_dht); Serial.println(" %");
  Serial.print("Sensação térmica: "); Serial.print(sensacao_termica); Serial.println(" °F\n");

  Serial.println("===== SENSOR MPU6050 =====");
  Serial.print("Aceleração (m/s²) - X: "); Serial.print(aceleracao.acceleration.x);
  Serial.print(" | Y: "); Serial.print(aceleracao.acceleration.y);
  Serial.print(" | Z: "); Serial.println(aceleracao.acceleration.z);

  Serial.print("Giroscópio (°/s) - X: "); Serial.print(giroscopio.gyro.x * RAD_TO_DEG);
  Serial.print(" | Y: "); Serial.print(giroscopio.gyro.y * RAD_TO_DEG);
  Serial.print(" | Z: "); Serial.println(giroscopio.gyro.z * RAD_TO_DEG);

  Serial.print("Temperatura MPU: "); Serial.print(temp_mpu.temperature); Serial.println(" °C");

  Serial.println("======================================\n");
}
