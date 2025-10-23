#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <Adafruit_INA219.h>

// ---------- DS18B20 ----------
#define DS18B20_1_PIN 2
#define DS18B20_2_PIN 3
#define DS18B20_3_PIN 4

OneWire oneWire1(DS18B20_1_PIN);
DallasTemperature sensor1(&oneWire1);

OneWire oneWire2(DS18B20_2_PIN);
DallasTemperature sensor2(&oneWire2);

OneWire oneWire3(DS18B20_3_PIN);
DallasTemperature sensor3(&oneWire3);

// ---------- INA219 ----------
Adafruit_INA219 ina219;

// ---------- Controle Térmico ----------
#define resistor 5   // Gate do MOSFET

// ---------- Configuração ----------
#define ENDERECO_SUPRIMENTO 1  // Endereço para identificar este módulo
#define SERIAL_BAUDRATE 115200

void setup() 
{
  Serial.begin(SERIAL_BAUDRATE);

  // Inicializa sensores
  sensor1.begin();
  sensor2.begin();
  sensor3.begin();
  digitalWrite(resistor, LOW); // garante que MOSFET inicia desligado

  if (!ina219.begin()) 
  {
    Serial.println("Falha ao encontrar INA219!");
    while (1) delay(10);
  }

  ina219.setCalibration_32V_2A();

  delay(500);
}

void loop() 
{
  // ---------- Leitura dos sensores ----------
  float busVoltage = ina219.getBusVoltage_V();
  float current = ina219.getCurrent_mA();

  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  sensor3.requestTemperatures();

  float temp1 = sensor1.getTempCByIndex(0);
  float temp2 = sensor2.getTempCByIndex(0);
  float temp3 = sensor3.getTempCByIndex(0);

  bool t1_valida = (temp1 > -100.0 && temp1 < 125.0);
  bool t2_valida = (temp2 > -100.0 && temp2 < 125.0);
  bool t3_valida = (temp3 > -100.0 && temp3 < 125.0);

  bool abaixo_25 = (t1_valida && temp1 < 25.0) ||
                   (t2_valida && temp2 < 25.0) ||
                   (t3_valida && temp3 < 25.0);

  bool acima_40 = (t1_valida && temp1 >= 40.0) ||
                  (t2_valida && temp2 >= 40.0) ||
                  (t3_valida && temp3 >= 40.0);

  if (abaixo_25)
  {
      digitalWrite(resistor, HIGH);
      digitalWrite(led, HIGH);
      Serial.println("MOSFET: LIGADO");
  }
  else if (acima_40)
  {
      digitalWrite(resistor, LOW);
      digitalWrite(led, LOW);
      Serial.println("MOSFET: DESLIGADO");
  }

  // ---------- Monta e envia mensagem ----------
  // Formato: <endereco>:<temp1>:<temp2>:<temp3>:<tensao>:<corrente>\n
  Serial.print(ENDERECO_SUPRIMENTO);
  Serial.print(":");
  Serial.print(temp1, 2);
  Serial.print(":");
  Serial.print(temp2, 2);
  Serial.print(":");
  Serial.print(temp3, 2);
  Serial.print(":");
  Serial.print(busVoltage, 2);
  Serial.print(":");
  Serial.println(current, 2);  // termina com \n

  delay(2000);
}
