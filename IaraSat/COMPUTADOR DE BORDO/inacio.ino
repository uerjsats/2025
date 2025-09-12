//Biblioteca para comunicação I2C
#include <Wire.h> 
#define sda 42 
#define scl 41
//Biblioteca para as funções do Arduino
#include <Arduino.h>

// Biblioteca do sensor BMP280
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; //'sensor_bmp' variável para chamar o sensor

// Biblioteca do sensor DHT
#include <DHT.h>
//Pino do sensor DHT
#define pino_dht 48
//Tipo de sensor DHT (DHT11 ou DHT12)
#define tipo_dht DHT22 
//'dht' variável para chamar o sensor e configurando em relação ao pino e o tipo
DHT dht(pino_dht, tipo_dht);

//Biblioteca do sensor MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
Adafruit_MPU6050 mpu;

//Pressão a nível do mar hpa
#define pressao_mar 1013.25

void setup()
{
    // Iniciando a comunicação serial com frequência de 9600Hz/115200Hz
    Serial.begin(115200); 
      if(!Serial)
      {
        delay(10);
      }
    //configurando os pinos de comunicação SDA e SCL
      Wire.begin (sda, scl); 
    //Inicializando o Sensor BMP no endereço I2C padrão (0x76)
      if(!bmp.begin(0x76))
      {
        Serial.println("Falha ao inicializar o Sensor BMP280!");
          while (1);
      }
      else
      {
          Serial.println("Sensor BMP280 iniciado com sucesso!");
      }
    //Configurando o sensor BMP280
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,  // Modo de medicao normal
                    Adafruit_BMP280::SAMPLING_X2,  // Oversampling da temperatura
                    Adafruit_BMP280::SAMPLING_X16, // Oversampling da pressao
                    Adafruit_BMP280::FILTER_X16,   // Filtro
                    Adafruit_BMP280::STANDBY_MS_500); // Tempo de espera em modo standby
  
    //inicializa o sensor DHT 
    dht.begin();
    //Inicializando o sensor MPU6050
    if(!mpu.begin(0x68))
    {
      Serial.println("Falha ao inicializar o sensor MPU6050");
    }
    else
    {
      Serial.println("Sensor MPU6050 inicializado com sucesso!");
    }
    // Definindo amplitude do giroscopio
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Faixa de giroscópio definida para: ");
    switch (mpu.getGyroRange()) 
    {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
    }

  // Definindo filtro de largura de banda
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    Serial.print("Largura de banda do filtro definida para: ");
    switch (mpu.getFilterBandwidth()) 
    {
      case MPU6050_BAND_260_HZ:
        Serial.println("260 Hz");
        break;
      case MPU6050_BAND_184_HZ:
        Serial.println("184 Hz");
        break;
      case MPU6050_BAND_94_HZ:
        Serial.println("94 Hz");
        break;
      case MPU6050_BAND_44_HZ:
        Serial.println("44 Hz");
        break;
      case MPU6050_BAND_21_HZ:
        Serial.println("21 Hz");
        break;
      case MPU6050_BAND_10_HZ:
        Serial.println("10 Hz");
        break;
      case MPU6050_BAND_5_HZ:
        Serial.println("5 Hz");
        break;
    }

    Serial.println("");
}


void loop()
{
  delay(2000); // Dois seguntos para iniciar cada leitura
    
  //Leitura do sensor BMP
  float temperatura_bmp = bmp.readTemperature();
  float pressao_bmp = bmp.readPressure();
  float altitude_bmp = bmp.readAltitude(pressao_mar);
  
  //Leitura do sensor DHT
  float temp_dht = dht.readTemperature(true); //Converte a temperatura em Fahrenheit
  float humid_dht = dht.readHumidity();
  
  if(isnan(temp_dht) || isnan(humid_dht))
  {
    //Se houver falha na leitura, ela retorna no início do loop e teanta novamente (return)
  	Serial.print("Falha ao ler o dados do sensor DHT!");
    return;
  }
  else
  {
    Serial.print("Dados do sensor DHT encontrado!");
  }
  
  //Sensação térmica DHT
  float sens_term_f = dht.computeHeatIndex(temp_dht, humid_dht); // Calcula sensacao termica em Fahrenheit (por padrao)
  
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
 
  /* Imprimindo valores */  
  Serial.print("Temperatura BMP: ");
  Serial.println(temperatura_bmp);
  Serial.print("hpa, Pressão BPM: ");
  Serial.println(pressao_bmp / 100.0F); //converte a pressão para hpa
  Serial.print("m, Altitude: ");
  Serial.println(altitude_bmp);
  
  Serial.print("Humidade: ");
  Serial.println(humid_dht);
  Serial.print("Sensação térmica (Fahreheit): ");
  Serial.println(sens_term_f);
  
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.println("");
}
