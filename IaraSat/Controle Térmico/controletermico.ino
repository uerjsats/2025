//Bibliotecas
#include <DallasTemperature.h>
#include <OneWire.h>

//Pino de Hardware
#define resistor 5
#define led 21
#define ONE_WIRE_BUS 15

//Inicia a biblioteca para pegar a temperatura
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);


void setup() {

  pinMode(resistor, OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);
  Serial.begin(115200);
  sensors.begin();

}

void loop() {

  float temperatura;

  temperatura = coletartemperatura();
  controle(temperatura);
  
}

float coletartemperatura(){

  //Pegar temperatura
  sensors.requestTemperatures(); 
  float temperaturaC = sensors.getTempCByIndex(0);
  Serial.print(temperaturaC);
  Serial.println("ºC");
  delay(5000);
  return temperaturaC;
  
}

void controle(float temperatura){
   //Controle da temperatura quando 0 Graus(Ainda em testes)
   String comando = Serial.read();
  if (temperatura == 0 || comando == "1"){
      //Liga
      digitalWrite(resistor, HIGH);
      digitalWrite(led, HIGH);
      delay(1000);

  }else if(comando == "0" || temperatura >= 40){
    //Desliga
    digitalWrite(resistor, LOW);
    digitalWrite(led, LOW);
    
  }
  delay(1000);
}
