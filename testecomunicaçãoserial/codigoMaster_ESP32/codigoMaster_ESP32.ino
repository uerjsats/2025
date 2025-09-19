#include <Arduino_MKRENV.h>
#include <MKRENV.h>
// Endereço 
int enderecoMaster = 42;
unsigned long tempoInicial;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Iniciando Testes: ");
}

void loop() {
  if (Serial1.available() > 0 || Serial.available() > 0) {
    String ordem = Serial.readStringUntil('\n');
    Serial.print(ordem);
    if (ordem == "1") {
      enviarOrdemslave("Teste de Comunição Serial", 1);
      aguardarResposta();
    }
  }
}

void aguardarResposta() {
  tempoInicial = millis();
  while (millis() - tempoInicial < 5000) {
    if (receberRespostaslave()) {
      break;
    }
  }
}

bool receberRespostaslave() {
  if (Serial1.available() > 0 || Serial.available() > 0) {
    String dadosRecebidos;
    if (Serial1.available() > 0) {
      dadosRecebidos = Serial1.readStringUntil('\n');
    } else {
      dadosRecebidos = Serial.readStringUntil('\n');
    }

    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();

    switch (enderecoRecebido) {
      case 4:
        processarDados("Arduino", dadosRecebidos.substring(1));
        return true;
      default:
        return false;
    }
  }
  return false;
}

void processarDados(String slave, String dados) {
  Serial.println("Dados recebidos do " + slave + " : " + dados);
  Serial1.println("Dados recebidos do" + slave + " : " + dados);  
}

void enviarOrdemslave(String dados, int endereco) {
  Serial1.print(endereco);
  Serial1.print(":");
  Serial1.println(dados);

  Serial.print(endereco);
  Serial.print(":");
  Serial.println(dados);
}