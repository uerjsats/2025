#include <Wire.h>

const int eepromAddress = 0x50; // Trocar pelo endereço do scanner I2C

void writeEEPROM(int address, byte data) {
  Wire.beginTransmission(eepromAddress);
  Wire.write((address >> 8) & 0xFF); // Parte alta do endereço
  Wire.write(address & 0xFF);       // Parte baixa do endereço
  Wire.write(data);
  Wire.endTransmission();
  delay(5); // Tempo necessário para a gravação
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Escrevendo na EEPROM...");

  int endereco = 0;  // Endereço de memória
  byte valor = 42;   // Valor a ser gravado

  writeEEPROM(endereco, valor);

  Serial.println("Dados gravados com sucesso!");
}

void loop() {
  // Nada aqui
}
