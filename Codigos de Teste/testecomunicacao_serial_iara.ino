#define ENDERECO_CONTROLE 3
String msg = "";
String ultimaMsg = "";

void setup() {
  Serial.begin(115200);
}

void loop() {
  mensagem();
}

void mensagem(){
  if (Serial.available()) {
     msg = Serial.readStringUntil('\n');
     msg.trim();
     if (msg != ultimaMsg && msg.length() > 0) { // só printa se mudou
         Serial.print(ENDERECO_CONTROLE);
         Serial.print(":");
         Serial.println(msg);
         ultimaMsg = msg;
     }
  }
}
