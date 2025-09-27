#include <Servo.h>
#include <Wire.h>

// --- ESC (opcional, pode remover se não usar agora) ---
Servo esc;
const int escPin = 2;

// --- LDR ---
int sensorLDR = A3;
int sensorLDR2 = A2;
int tolerancia = 10;
double menorLuz = 1023.0;  // começa no máximo possível do ADC
double anguloMaisClaro = 0.0;

// --- MPU6050 ---
#define MPU_ADDRESS 0x68
double GYRO_ERROR = 0.0;
double yawAngle = 0, yawAngularSpeed = 0;

// --- Filtro simples ---
const int numReadings = 20;
double readings[numReadings];
int readIndex = 0;
double total = 0, rollingAvg = 0;

// --- Tempo ---
long timeCur, timePrev;

// --- Funções ---
int16_t readMPU() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 2, true);
  return Wire.read() << 8 | Wire.read();
}

void calibrateMPU() {
  GYRO_ERROR = 0;
  for (int i = 0; i < 200; i++) {
    GYRO_ERROR += readMPU();
    delay(20);
  }
  GYRO_ERROR /= 200.0;
}

void setup() {
  Serial.begin(9600);

  // Inicializa ESC (opcional)
  esc.attach(escPin);
  esc.writeMicroseconds(1000);

  // Inicializa LDR
  pinMode(sensorLDR, INPUT);
  pinMode(sensorLDR2, INPUT);

  // Inicializa filtro
  for (int i = 0; i < numReadings; i++) readings[i] = 0;

  // Inicializa MPU
  Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x10); // 1000 dps
  Wire.endTransmission(true);

  delay(3000);
  calibrateMPU();

  timeCur = millis();
  pinMode(11, OUTPUT);
}

void loop() {
  int leitura = analogRead(sensorLDR);
  int leitura2 = analogRead(sensorLDR2);
  if (millis() - timeCur > 10) {
    timePrev = timeCur;
    timeCur = millis();

    // Atualiza ângulo
    yawAngularSpeed = ((double)readMPU() - GYRO_ERROR) / 32.8;
    yawAngle += yawAngularSpeed * (timeCur - timePrev) / 1000.0;

    while (yawAngle <= -180) yawAngle += 360;
    while (yawAngle > 180) yawAngle -= 360;

    // Atualiza média móvel
    total -= readings[readIndex];
    readings[readIndex] = yawAngularSpeed;
    total += readings[readIndex];
    readIndex++;
    if (readIndex >= numReadings) readIndex = 0;
    rollingAvg = total / numReadings;
  }

  // Salva ângulo se a luz aumentou (resistência diminuiu)
  //if (leitura < menorLuz) {
  //  menorLuz = leitura;
   // anguloMaisClaro = yawAngle;
   // Serial.print("Máximo de luz: ");
   // Serial.print(menorLuz);
   // Serial.print(" no ângulo: ");
   // Serial.println(anguloMaisClaro);
  //}
  Serial.print("LDR 1: ");
  Serial.print(leitura);
  Serial.print(" / ");
  Serial.print("LDR2: ");
  Serial.print(leitura2);
  Serial.print(" / ANGULO: ");
  Serial.println(yawAngle);

  delay(500);
  // Debug opcional: ângulo atual
  // Serial.print("Ângulo atual: ");
  // Serial.println(yawAngle);
  if (abs(leitura - leitura2)<tolerancia){
    if ((leitura + leitura2)/2<menorLuz){
      anguloMaisClaro = yawAngle;
      menorLuz= (leitura + leitura2)/2; 

       Serial.print("Máximo de luz: ");
       Serial.print(menorLuz);
       Serial.print(" no ângulo: ");
       Serial.println(anguloMaisClaro);

      digitalWrite(11, HIGH);
      delay(1000);
    }
  }else{
    digitalWrite(11, LOW);
  }
}