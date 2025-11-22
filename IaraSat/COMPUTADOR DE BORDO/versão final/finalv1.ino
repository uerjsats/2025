#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "float16.h"

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

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

double txNumber;
bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr);  

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#define SEALEVELPRESSURE_HPA (950)
#define DHTPIN 48
#define DHTTYPE DHT22
#define TIMEZONE_OFFSET -3

Adafruit_BME280 bme;
DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;

unsigned long startTime;
unsigned long lastTxTime = 0;
const unsigned long txInterval = 2000; // envia a cada 2s

//pacotes de dados
String powerSupplyData = " ";
String ACData = " ";
String MData= " ";

// Flags de atualização
bool powerSupplyUpdated = false;
bool MDataUpdated = false;
bool ACDataUpdated = false;

class EE24CXXX {
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

void EE24CXXX::write(unsigned int eeaddress, unsigned char *data, unsigned int data_len) {
    while (data_len > 0) {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));
        Wire.write((int)(eeaddress & 0xFF));

        byte bytesToWrite = min(data_len, (unsigned int)16);
        for (byte i = 0; i < bytesToWrite; i++) {
            Wire.write(data[i]);
        }

        Wire.endTransmission();
        eeaddress += bytesToWrite;
        data += bytesToWrite;
        data_len -= bytesToWrite;

        delay(5);
    }
}

void EE24CXXX::read(unsigned int eeaddress, unsigned char *data, unsigned int data_len) {
    while (data_len > 0) {
        Wire.beginTransmission(_device_address);
        Wire.write((int)(eeaddress >> 8));
        Wire.write((int)(eeaddress & 0xFF));
        Wire.endTransmission();

        byte bytesToRead = min(data_len, (unsigned int)28);
        Wire.requestFrom(_device_address, bytesToRead);

        for (byte i = 0; i < bytesToRead && Wire.available(); i++) {
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

struct sensorsData {
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

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600, SERIAL_8N1, 45, 46);

    if (!bme.begin(0x76)) {
        Serial.println("Erro ao inicializar o BME280!");
    }

    dht.begin();
    Wire.begin();

    if (!mpu.begin()) {
        Serial.println("Erro ao inicializar o MPU6050!");
    }

    startTime = millis();

    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

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

void loop() {

    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        gps.encode(c);
    }

    Radio.IrqProcess();
    
    //Contador

    if (millis() - lastTxTime >= txInterval && lora_idle) {
        sensorsData dados;
        unsigned short elapsedTime = (millis() - startTime) / 1000;
        dados.seconds = elapsedTime;
        
        //Dados DHT
        dados.temperatureDHT = dht.readTemperature();
        if (isnan(dados.temperatureDHT)) dados.temperatureDHT = 0.0;

        dados.humidityDHT = dht.readHumidity();
        if (isnan(dados.humidityDHT)) dados.humidityDHT = 0.0;
        
        //Dados do BME
        dados.pressure = bme.readPressure() / 100.0F;
        static float refPressure = 0;
        if (refPressure == 0) 
        {
            refPressure = dados.pressure;  
            //Serial.print("Pressão base calibrada: ");
            //Serial.println(refPressure);
        }

        dados.altitude = bme.readAltitude(refPressure);
        
        //Dados GPS
        dados.latitude = gps.location.lat();
        dados.longitude = gps.location.lng();
        dados.sats = gps.satellites.value();

        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);
        
        //Dados GY
        dados.accelX = a.acceleration.x;
        dados.accelY = a.acceleration.y;
        dados.accelZ = a.acceleration.z;

        receberRespostaslave();
        
        // Escrita na EEPROM
        // Só irão ser escritos de 20 em 20 bytes, e irá cessar no máximo da memoria (1484 bytes)
        if ((currentAddress <= 1480) && (dados.altitude >= 500.00)) {

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

        if (powerSupplyUpdated || MDataUpdated || ACDataUpdated) {
             //Monta o pacote LoRa
            snprintf(&txpacket[2], BUFFER_SIZE - 2,
                "%u:%.2f:%.2f:%.2f:%.2f:%d:%.6f:%.6f:%.2f:%.2f:%.2f:%s:%s:%s",
                dados.seconds, dados.temperatureDHT, dados.humidityDHT, dados.altitude,
                dados.pressure, dados.sats, dados.latitude, dados.longitude,
                dados.accelX, dados.accelY, dados.accelZ,powerSupplyData.c_str(),ACData.c_str(),
                MData.c_str()
            );
            powerSupplyUpdated = false;
            MDataUpdated = false;
            ACDataUpdated = false;
        } 
        else{
            //Monta o pacote LoRa
          snprintf(&txpacket[2], BUFFER_SIZE - 2,
                "%u:%.2f:%.2f:%.2f:%.2f:%d:%.6f:%.6f:%.2f:%.2f:%.2f",
                dados.seconds, dados.temperatureDHT, dados.humidityDHT, dados.altitude,
                dados.pressure, dados.sats, dados.latitude, dados.longitude,
                dados.accelX, dados.accelY, dados.accelZ
          );
        }

        //Envia dados para estação base
        if (txpacket[1] == DEST_ADDRESS) {
            Radio.Send((uint8_t *)txpacket, strlen((char *)txpacket));
            lora_idle = false;
            lastTxTime = millis();
        }
        
    }
}

//Recebe os dados dos subsistemas
int receberRespostaslave() 
{
    if (!Serial.available()) return -1;

    // Lê a linha inteira de uma vez
    String linha = Serial.readStringUntil('\n');
    linha.trim();  // Remove \r e espaços

    if (linha.length() == 0) return -1;

    // Verifica se existe o separador
    int idx = linha.indexOf(':');
    if (idx == -1) {
        return -1; // Pacote inválido
    }

    String enderecoStr = linha.substring(0, idx);
    int endereco = enderecoStr.toInt();

    // Dados depois do endereço
    String dados = linha.substring(idx + 1);

    switch (endereco) {

        case 1: // Suprimento
            powerSupplyData = dados;
            powerSupplyUpdated = true;
            break;

        case 2: // Missao
            MData = dados;
            MDataUpdated = true;
            break;

        case 3: // Controle de Atitude
            ACData = dados;
            ACDataUpdated = true;
            break;

        default:
            return -1;
    }

    return 0;
}

void enviarOrdemslave(int Buscador) 
{
  switch(Buscador){
    case 1:
      Serial.println("1"); // Controle de atitude inicializado
      break;
    
    case 2:
      Serial.println("2"); // Pedir Azimute 
      break;

    // CASE 3 é outra função 
    
    case 4:
      Serial.println("4"); //orienta pra luz
      break;
    
    case 5: 
      Serial.println("5"); //parar controle de atitude
      break;
    
    case 6: 
      Serial.println("6"); //abrir painéis 
      break;

    case 7:
      Serial.println("7"); //abrir antena
      break;

    case 8:
      Serial.println("8"); //Fecha a antena
      break;
    
    case 9:
      Serial.println("9");
      break;

    default:
      Serial.println("Ordem inválida: " + String(Buscador));
      break;  
  }

}
//AJUSTAR ESSA FUNÇÃO
void enviarAngulosParaSlave(float ang1, float ang2) //Função pra enviar ângulo para o controle de atitude
{
    String comando = "3:" + String(ang1, 2) + ":" + String(ang2, 2);
    Serial.println(comando); // envia via serial
    Serial.println("Enviando ângulos ao controle de atitude...");
}

void OnTxDone() {
    lora_idle = true;
    Radio.Rx(0);
}

void OnTxTimeout() {
    Radio.Sleep();
    lora_idle = true;
    Radio.Rx(0);
}

//Recebe Telecomando
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssiValue, int8_t snr) {
    if (size < 2) return;

    uint8_t sender = payload[0];
    uint8_t receiver = payload[1];

    if (receiver != MY_ADDRESS || sender != DEST_ADDRESS) {
        lora_idle = true;
        Radio.Rx(0);
        return;
    }

    memset(rxpacket, 0, BUFFER_SIZE);
    memcpy(rxpacket, &payload[2], size - 2);
    rxpacket[size - 2] = '\0';
    Radio.Sleep();

    if (strcmp(rxpacket, "1") == 0) { //COMANDO PRA INICIAR CONTROLE DE ATITUDE
        enviarOrdemslave(1);
    }
    else if(strcmp(rxpacket, "2") == 0) //COMANDO PRA RECEBER AZIMUTE
    {
        enviarOrdemslave(2);
    }
    else if(strcmp(rxpacket, "3") == 0) //COMANDO DOS ÂNGULOS
    {
        float ang1 = 0, ang2 = 0;
        sscanf(rxpacket, "3:%f:%f", &ang1, &ang2);
        Serial.print("Recebidos ângulos: ");
        Serial.print(ang1);
        Serial.print(" e ");
        Serial.println(ang2);

        enviarAngulosParaSlave(ang1, ang2);
    }
    else if(strcmp(rxpacket, "4") == 0) //ORIENTA PRA LUZ
    {
        enviarOrdemslave(4);
    }
    else if(strcmp(rxpacket, "5") == 0) //PARAR CONTROLE DE ATITUDE
    {
        enviarOrdemslave(5);
    }
    else if(strcmp(rxpacket, "6") == 0) // ABRIR PAINÉIS
    {
        enviarOrdemslave(6);
    }
    else if(strcmp(rxpacket, "7") == 0) // ABRIR ANTENA
    {
        enviarOrdemslave(7);
    }
    else if(strcmp(rxpacket, "8") == 0) // FECHA ANTENA
    {
        enviarOrdemslave(8);
    }
    else if(strcmp(rxpacket, "9") == 0) // MISSÃO
    {
        enviarOrdemslave(9);
    }

    lora_idle = true;
}
