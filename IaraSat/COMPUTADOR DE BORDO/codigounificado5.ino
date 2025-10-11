#include <Wire.h>
#include <Arduino.h>

#include <Adafruit_BME280.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

#include "LoRaWan_APP.h"
#include "float16.h"

#include "heltec.h"


// ========== Definições ==========
#define SDA_PIN 41
#define SCL_PIN 42

#define DHT_PIN 48
#define DHT_TYPE DHT22 

#define GPS_RX 45
#define GPS_TX 46

#define PRESSAO_NIVEL_MAR 1013.25 // hPa
#define HELTEC_BOARD 1
#define SHOW_CLK_TYPE 0

// ========== Objetos dos sensores ==========
Adafruit_BME280 bme; // Sensor de pressão/temperatura
DHT dht(DHT_PIN, DHT_TYPE); // Sensor de umidade/temperatura
Adafruit_MPU6050 mpu; // Sensor de aceleração/giroscópio
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;


//Comunicação Serial
int enderecoMaster = 42;
unsigned long tempoInicial;

// ========== Rádio LoRa 
#define RF_FREQUENCY 915000000 // Hz
#define TX_OUTPUT_POWER 20

#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 300

#define MY_ADDRESS 43 //Endereço do rádio do satélite
#define DEST_ADDRESS 42 //Endereço da estação base

String powerSupplyData = "";

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;
bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr); 

unsigned long startTime;
unsigned long lastTxTime = 0;
const unsigned long txInterval = 2000; // envia a cada 2s

//============EEPROM========
class EE24CXXX 
{
private:
    byte _device_address;

public:
    EE24CXXX(byte device_address) : _device_address(device_address) {}
    void write(unsigned int eeaddress, unsigned char *data, unsigned int data_len);
    void read(unsigned int eeaddress, unsigned char *data, unsigned int data_len);

    template <class T>
    int write(unsigned int eeaddress, const T &value);
    template <class T>
    int read(unsigned int eeaddress, T &value);
};

void EE24CXXX::write(unsigned int eeaddress, unsigned char *data, unsigned int data_len) 
{
    while (data_len > 0) 
    {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));
        Wire.write((int)(eeaddress & 0xFF));

        byte bytesToWrite = min(data_len, (unsigned int)16);
        for (byte i = 0; i < bytesToWrite; i++) 
        {
            Wire.write(data[i]);
        }

        Wire.endTransmission();
        eeaddress += bytesToWrite;
        data += bytesToWrite;
        data_len -= bytesToWrite;

        delay(5);
    }
}

void EE24CXXX::read(unsigned int eeaddress, unsigned char *data, unsigned int data_len) 
{
    while (data_len > 0) 
    {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));
        Wire.write((int)(eeaddress & 0xFF));
        Wire.endTransmission();

        byte bytesToRead = min(data_len, (unsigned int)28);
        Wire.requestFrom(_device_address, bytesToRead);

        for (byte i = 0; i < bytesToRead && Wire.available(); i++) 
        {
            data[i] = Wire.read();
        }

        data += bytesToRead;
        eeaddress += bytesToRead;
        data_len -= bytesToRead;
    }
}

template <class T> int EE24CXXX::write(unsigned int eeaddress, const T &value) {
    write(eeaddress, (unsigned char *)&value, sizeof(T));
    return sizeof(T);
}

template <class T> int EE24CXXX::read(unsigned int eeaddress, T &value) {
    read(eeaddress, (unsigned char *)&value, sizeof(T));
    return sizeof(T);
}

EE24CXXX eeprom(0x50);
int currentAddress = 0;

struct sensorsData 
{
    unsigned short seconds;
    float temperatureDHT;
    float humidityDHT;
    float pressure;
    int sats;
    float altitude;
    float latitude;
    float longitude;
    float accelX;
    float accelY;
    float accelZ;
};


// ========== Setup ==========
void setup() 
{
  Serial.begin(115200);
  while (!Serial); // Aguarda inicialização da porta serial
  
  gpsSerial.begin(9600, SERIAL_8N1,45,46);
  
  Wire.begin(SDA_PIN, SCL_PIN);
    
  // ----- BME280 -----
  if (!bme.begin(0x76)) 
  {
    Serial.println("Erro ao inicializar o sensor BME280 no endereço 0x76!");
  }
  else
  {
      Serial.println("Sensor BME inicializado no endereço 0x76");
  }
  // ----- DHT22 -----
  dht.begin();

  // ----- MPU6050 -----
  if (!mpu.begin(0x68)) 
  {
    Serial.println("Erro ao inicializar o sensor MPU6050!");
  } 
  else 
  {
    Serial.println("Sensor MPU6050 inicializado com sucesso.");
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    Serial.print("Faixa do giroscópio: ");
    switch (mpu.getGyroRange()) 
    {
      case MPU6050_RANGE_250_DEG: Serial.println("±250 °/s"); break;
      case MPU6050_RANGE_500_DEG: Serial.println("±500 °/s"); break;
      case MPU6050_RANGE_1000_DEG: Serial.println("±1000 °/s"); break;
      case MPU6050_RANGE_2000_DEG: Serial.println("±2000 °/s"); break;
    }

    Serial.print("Filtro de banda definido para: ");
    switch (mpu.getFilterBandwidth()) 
    {
      case MPU6050_BAND_260_HZ: Serial.println("260 Hz"); break;
      case MPU6050_BAND_184_HZ: Serial.println("184 Hz"); break;
      case MPU6050_BAND_94_HZ: Serial.println("94 Hz"); break;
      case MPU6050_BAND_44_HZ: Serial.println("44 Hz"); break;
      case MPU6050_BAND_21_HZ: Serial.println("21 Hz"); break;
      case MPU6050_BAND_10_HZ: Serial.println("10 Hz"); break;
      case MPU6050_BAND_5_HZ: Serial.println("5 Hz"); break;
    }
  }
  tempoInicial = millis();
  Serial.println("\nLEITURA DOS DADOS DOS SENSORES:\n");

  //================= Radio =========================

  startTime = millis();

    Mcu.begin(HELTEC_BOARD, SHOW_CLK_TYPE);

    txNumber = 0;

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    Radio.Rx(0);
}

// ========== Loop ==========
void loop() 
{
  delay(5000);
  //Radio
  Radio.IrqProcess();

  //Inicialização da Variável da Struct
  sensorsData dados = {0}; // tudo zerado

  // ===== Leitura automática dos sensores =====
  leituraDHT22(dados);    // temperatura e umidade
  leituraBME280(dados);   // pressão e altitude
  leituraMPU6050(dados);  // aceleração
  leituraGPS(dados);      // latitude, longitude, sats (se GPS fix)

   //Contador do rádio
  if (millis() - lastTxTime >= txInterval && lora_idle) 
  {
      unsigned short elapsedTime = (millis() - startTime) / 1000;
      dados.seconds = elapsedTime;


      // Escrita na EEPROM
      // Só irão ser escritos de 20 em 20 bytes, e irá cessar no máximo da memoria (1484 bytes)
      if ((currentAddress <= 1480) && (dados.altitude >= 200.00)) 
      {

          // float 4 bytes -> float 2 bytes
          eeprom.write(currentAddress, float16(dados.seconds)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.temperatureDHT)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.humidityDHT)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.pressure)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.altitude)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.latitude)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.longitude)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.accelX)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.accelY)); currentAddress += 2;
          eeprom.write(currentAddress, float16(dados.accelZ)); currentAddress += 2;

      }

      memset(txpacket, 0, BUFFER_SIZE);
      txpacket[0] = MY_ADDRESS;
      txpacket[1] = DEST_ADDRESS;
        
      //Monta o pacote LoRa
      snprintf(&txpacket[2], BUFFER_SIZE - 2, "%u:%.2f:%.2f:%.2f:%.2f:%d:%.6f:%.6f:%.2f:%.2f:%.2f:%s",
                dados.seconds, dados.temperatureDHT,
                dados.humidityDHT, dados.altitude,
                dados.pressure, dados.sats, dados.latitude, dados.longitude,
                dados.accelX, dados.accelY, dados.accelZ, powerSupplyData.c_str());
        
      //Envia dados para estação base
      if (txpacket[1] == DEST_ADDRESS) 
      {
          Radio.Send((uint8_t *)txpacket, strlen((char *)txpacket));
          lora_idle = false;
          lastTxTime = millis();
      }
  }

  delay(5000); // pequeno atraso para não lotar o monitor serial

}

// ----- Leitura MPU6050 -----
void leituraMPU6050(sensorsData &dados) {
    sensors_event_t aceleracao, giroscopio, temp_mpu;
    mpu.getEvent(&aceleracao, &giroscopio, &temp_mpu);

    Serial.print(":"); Serial.print(aceleracao.acceleration.x);
    Serial.print(":"); Serial.print(aceleracao.acceleration.y);
    Serial.print(":"); Serial.print(aceleracao.acceleration.z);

    Serial.print(":"); Serial.print(giroscopio.gyro.x * RAD_TO_DEG);
    Serial.print(":"); Serial.print(giroscopio.gyro.y * RAD_TO_DEG);
    Serial.print(":"); Serial.print(giroscopio.gyro.z * RAD_TO_DEG);

    // Atualiza struct
    dados.accelX = aceleracao.acceleration.x;
    dados.accelY = aceleracao.acceleration.y;
    dados.accelZ = aceleracao.acceleration.z;
}


// ----- Leitura BME280 -----
void leituraBME280(sensorsData &dados) {
    float temp_bme = bme.readTemperature();
    float pressao_bme = bme.readPressure() / 100.0F;
    float altitude_bme = bme.readAltitude(PRESSAO_NIVEL_MAR);

    Serial.print(":"); Serial.print(temp_bme); 
    Serial.print(":"); Serial.print(pressao_bme); 
    Serial.print(":"); Serial.print(altitude_bme); 

    // Atualiza struct
    dados.pressure = pressao_bme;
    dados.altitude = altitude_bme;
}

// ----- Leitura DHT22 -----
void leituraDHT22(sensorsData &dados) {
    float temp_dht_c = dht.readTemperature();
    float umidade_dht = dht.readHumidity();

    if (isnan(temp_dht_c) || isnan(umidade_dht)) {
        Serial.println("Erro ao ler dados do sensor DHT22.\n");
        dados.temperatureDHT = 0.0;
        dados.humidityDHT = 0.0;
        return;
    }

    
    Serial.print(":"); Serial.print(temp_dht_c); 
    Serial.print(":"); Serial.println(umidade_dht);  

    // Atualiza struct
    dados.temperatureDHT = temp_dht_c;
    dados.humidityDHT = umidade_dht;
}


// ----- Leitura GPS -----
void leituraGPS(sensorsData &dados) {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());

        if (gps.location.isUpdated()) {

            Serial.print(":"); Serial.print(gps.location.lat(), 6);
            Serial.print(":"); Serial.print(gps.location.lng(), 6);
            Serial.print(":"); Serial.println(gps.satellites.value());

            if (!gps.location.isValid()) {
                Serial.println("Sem sinal...");
            }

            // Atualiza struct
            dados.latitude = gps.location.lat();
            dados.longitude = gps.location.lng();
            dados.sats = gps.satellites.value();
        }
    }
}


void aguardarResposta() 
{
  tempoInicial = millis();
  while (millis() - tempoInicial < 5000) 
  {
    if (receberRespostaslave()) 
    {
      break;
    }
  }
}

bool receberRespostaslave() 
{
  if (Serial.available() > 0) 
  {
    String dadosRecebidos;
    if (Serial.available() > 0) 
    {
      dadosRecebidos = Serial1.readStringUntil('\n');
    } 
    else 
    {
      dadosRecebidos = Serial.readStringUntil('\n');
    }

    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();

    switch (enderecoRecebido) 
    {
      case 4:
        processarDados("Arduino", dadosRecebidos.substring(1));
        return true;
      default:
        return false;
    }
  }
  return false;
}

void processarDados(String slave, String dados) 
{
  Serial.println("Dados recebidos do " + slave + " : " + dados);
  Serial1.println("Dados recebidos do" + slave + " : " + dados);  
}

void enviarOrdemslave(String dados, int endereco) 
{
  Serial1.print(endereco);
  Serial1.print(":");
  Serial1.println(dados);

  Serial.print(endereco);
  Serial.print(":");
  Serial.println(dados);

}

//RADIO
void OnTxDone() 
{
    lora_idle = true;
    Radio.Rx(0);
}

void OnTxTimeout() 
{
    Radio.Sleep();
    lora_idle = true;
    Radio.Rx(0);
}

//Recebe Telecomando
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr) 
{
    if (size < 2) return;

    uint8_t sender = payload[0];
    uint8_t receiver = payload[1];

    if (receiver != MY_ADDRESS || sender != DEST_ADDRESS) 
    {
        lora_idle = true;
        Radio.Rx(0);
        return;
    }

    memset(rxpacket, 0, BUFFER_SIZE);
    memcpy(rxpacket, &payload[2], size - 2);
    rxpacket[size - 2] = '\0';
    Radio.Sleep();

    if (strcmp(rxpacket, "1") == 0) 
    {
        Serial.println("1");
    }

    lora_idle = true;
}
