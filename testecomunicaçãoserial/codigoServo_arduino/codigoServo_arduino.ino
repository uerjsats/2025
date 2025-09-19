//unsigned long tempoInicial;

void setup() {

  Serial.begin(115200);

}

void loop() {
  bool comando = receberOrdemmaster(1);
  if(comando == true) { 
    Serial.println("Comando recebido! Esperando ");
    enviarRespostamaster("Tudo Certo !!!", 4);
  }
  delay(100);
  }
bool receberOrdemmaster(int endereco) {
  if (Serial.available() > 0) {
    String dadosRecebidos = Serial.readStringUntil('\n');
    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    if (enderecoRecebido == endereco) {
      String dados = dadosRecebidos.substring(2);
      Serial.println("Comando recebido do Master: ");
      Serial.print(dados);
      tempoInicial = millis();
      return true;
    }
    return false;
  }
}

void enviarRespostamaster(String dados, int endereco) {
  Serial.print(endereco);  
  Serial.print(" : ");
  Serial.println(dados);
}