#include <Wire.h>

const int eepromAddress = 0x50; // Endereço I2C da AT24C512

void readEEPROM(int address, char* buffer, int length) {
  Wire.beginTransmission(eepromAddress);
  Wire.write((address >> 8) & 0xFF); // Parte alta do endereço
  Wire.write(address & 0xFF);       // Parte baixa do endereço
  Wire.endTransmission();

  Wire.requestFrom(eepromAddress, length);
  for (int i = 0; i < length && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  char buffer[64]; // Buffer para armazenar os dados lidos
  
  // Lendo os dados da EEPROM a partir do endereço 0
  readEEPROM(0, buffer, sizeof(buffer));
  
  Serial.print("Dados lidos da EEPROM: ");
  Serial.println(buffer);
}

void loop() {
  // Nada aqui
}
