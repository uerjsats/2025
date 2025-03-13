#include <Wire.h>

const int eepromAddress = 0x50; // Endereço deverá ser modificado pelo esdereço obtido no scanner I2C

byte readEEPROM(int address) {
  Wire.beginTransmission(eepromAddress);
  Wire.write((address >> 8) & 0xFF); // Parte alta do endereço
  Wire.write(address & 0xFF);       // Parte baixa do endereço
  Wire.endTransmission();

  Wire.requestFrom(eepromAddress, 1); // Solicita 1 byte
  return Wire.read();
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Lendo da EEPROM...");

  int endereco = 0; // Endereço que queremos ler
  byte valorLido = readEEPROM(endereco);

  Serial.print("Valor lido: ");
  Serial.println(valorLido);
}

void loop() {
  // Nada aqui
}
