#define ENDERECO_RESPOSTA 1

String azimute = "23.45";  // valor fixo só para teste

void setup() {
  Serial.begin(115200);
  Serial.println("Slave pronto");
}

void loop() {
  // Verifica se recebeu algo pela serial
  if (Serial.available() > 0) {
    // Lê o comando vindo do Heltec
    String comando = Serial.readStringUntil('\n');
    comando.trim(); // remove espaços e quebras de linha

    // Se recebeu o comando "2", responde com o azimute
    if (comando == "2") {
      Serial.print(ENDERECO_RESPOSTA);
      Serial.print(":");
      Serial.println(azimute);
    }
  }
}
