//Variáveis Globais
long tempoInicial = 0; //Armazena o tempo do código
long tempoFinal = 0;
String dados = "";
float wSat = 0;
float alfaSat = 0;
float gyroX_offset = 0;

/*------------Roda de reação----------*/

//MPU 6050
//Biblioteca do sensor MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

//Função MPU
float angulo();
//Mapeamento de Hardware
#define LDR1 A0 //Pino do LDR1
#define LDR2 A1 //Pino do LDR2
#define SCL A5 //MPU
#define SCA A4 //MPU

//Função Controle de Atitude
void controleAtitude(float LDR1, float LDR2);

/*//Funções Comunicação Serial
bool receberOrdemmaster(int endereco); //Recebe o comando via Serial
void enviarRespostamaster(String dados, int endereco); //Devolve o que fez*/



void setup() 
{
  pinMode(LDR1, INPUT); //Declara LDR1 para entrada
  pinMode(LDR2, INPUT);  //Declara LDR2 para entrada
  Serial.begin(115200);
  Serial.println("Teste de Roda de Reação (Velocidade ângular | Ângulo): ");


  //Verifica se tem um MPU para conectar
  if (!mpu.begin()) 
  {
    dados = "Falha no MPU";
    Serial.println(dados);
    //enviarRespostamaster(dados, 1);
    while (1) 
    {
      delay(10);
    }
  }
  
  else
  {
    dados = "Tudo Certo no MPU !!!";
    Serial.println(dados);
    //enviarRespostamaster(dados, 1);
    //Inicialização da Largura de Banda para os dados de ângulo
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  }
  tempoInicial = millis();

}

void loop() 
{

  tempoFinal = millis();
  //Tempo de código
  if(Serial.available()>0)
  {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    if (/*receberOrdemmaster(1) == true ||*/ comando == "1")
    {
      float ldr1 = analogRead(LDR1) - 20;
      float ldr2 = analogRead(LDR2); 
      controleAtitude(ldr1, ldr2);
      dados = String(wSat, 2) + " | " + String(alfaSat, 2);
      /*enviarRespostamaster(dados, 2);*/
      if(tempoFinal - tempoInicial < 5000)
      {
        Serial.println(dados);
      }
    }
  }
  tempoInicial = tempoFinal;
}


float wangulo()
{

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float angulo = g.gyro.y;
  return angulo - gyroX_offset;

}

void controleAtitude(float LDR1, float LDR2)
{

  if(LDR1 == LDR2)
  {
    wSat = 0;
    alfaSat = 0;
  }

  else
  {
    wSat = wangulo();
    alfaSat += wSat * ((tempoFinal - tempoInicial)/1000.0);

  }

}
/*bool receberOrdemmaster(int endereco, String op) 
{
  if (Serial.available() > 0) 
  {
    String dadosRecebidos = Serial.readStringUntil('\n');
    int enderecoRecebido = dadosRecebidos.substring(0, 1).toInt();
    
    if (enderecoRecebido == endereco || op == "1") 
    {
      String dados = dadosRecebidos.substring(2);
      Serial.println("Comando recebido do Master: ");
      Serial.print(dados);
      if(dados == "1" || op == "1")
      {
        float ldr1 = analogRead(LDR1) - 20;
        float ldr2 = analogRead(LDR2);
        controleAtitude(ldr1, ldr2);

        return true;
      }
    return false;
    }
  }
}

void enviarRespostamaster(String dados, int endereco) 
{
  Serial.print(endereco);  
  Serial.print(" : ");
  Serial.println(dados);
}*/