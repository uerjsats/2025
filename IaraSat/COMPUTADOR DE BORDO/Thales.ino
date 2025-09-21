#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>

#define SDA 41
#define SCL 42

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL);
  gpsSerial.begin(9600, SERIAL_8N1);

  if (!mpu.begin(0x68)) {
    Serial.println("Falha ao inicializar o MPU6050!");
  }
  
  Serial.println("MPU6050 inicializado com sucesso!");

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  Serial.print("Faixa de giroscópio definida para: ");
  switch (mpu.getGyroRange()) {
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
  default:
    Serial.println("Faixa desconhecida");
    }

  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  Serial.print("Largura de banda do filtro definida para: ");
  switch (mpu.getFilterBandwidth()) {
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
    default:
      Serial.println("Banda desconhecida");
  }
}

void loop() {
  sensors_event_t a,g,temp;
  mpu.getEvent(&a, &g, &temp);

  while(gpsSerial.available()){ //O monitor serial precisar estar em 115200
    gps.encode(gpsSerial.read());

    if(gps.location.isUpdated()){
    Serial.println("Latitude: "); Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: "); Serial.println(gps.location.lng(),6);
    Serial.print("Altitude: "); Serial.println(gps.altitude.meters());
    Serial.print("Velocidade: "); Serial.println(gps.speed.kmph());
    Serial.print("Hora: "); Serial.print(gps.time.hour()); 
    Serial.print(":"); Serial.println(gps.time.minute());

    if(!gps.location.isValid()){
      Serial.println("Sinal: ");
      }
   }
  }

  Serial.println("Acelaração X: ");
  Serial.println(a.acceleration.x);
  Serial.println("Acelaração y: ");
  Serial.println(a.acceleration.y);
  Serial.println("Acelaração z: ");
  Serial.println(a.acceleration.z);

  Serial.println("Giroscopio X (deg/s)");
  Serial.println(g.gyro.x * 180 / PI);
  Serial.println("Giroscopio y");
  Serial.println(g.gyro.y * 180 / PI);
  Serial.println("Giroscopio z");
  Serial.println(g.gyro.z * 180 / PI);

  Serial.println("Temp:");
  Serial.println(temp.temperature);

  delay(1000);
}
