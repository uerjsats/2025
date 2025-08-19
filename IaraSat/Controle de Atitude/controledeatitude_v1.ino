#include <Servo.h>

Servo esc; // Objeto para controlar o ESC
const int potPin = A0; // Pino do potenciômetro
const int escPin = 2; // Pino do ESC

// ------MPU6050-----
#include <Wire.h>
#define MPU_ADDRESS 0x68   // I2C address of the MPU-6050
double GYRO_ERROR = 61.96; // rollingAvg error compensation
double yawAngle=0, yawAngularSpeed=0;


// ------GENERAL-----
long timeCur, timePrev, timeStart; 
const int numReadings= 20;
double readings[numReadings];
int readIndex = 0;
double total = 0, rollingAvg = 0;
double targetAttitude = 0;
int control = 0;
// ------------------
  

void setup() {
  esc.attach(escPin);
  esc.writeMicroseconds(1000);
  timeCur = millis();
  timeStart = timeCur;

// ------------------
  // Gyro setup
 Wire.begin();
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // wakes up the MPU-6050
  
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x10); // 1000dps full scale
  Wire.endTransmission(true);
  Serial.begin(9600);
  delay(3000);
  calibrateMPU();
}
// Read a yaw angular speed value
int16_t readMPU(){
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS,2,true);
  return Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

#define CALIBRATION_MEASUREMENTS_COUNT 200
void calibrateMPU(){
  GYRO_ERROR = 0;
  for(int i=0;i<CALIBRATION_MEASUREMENTS_COUNT;i++){
    GYRO_ERROR += readMPU();
    delay(20);
  }
  GYRO_ERROR = GYRO_ERROR/(double)CALIBRATION_MEASUREMENTS_COUNT;
  #if SERIAL_DEBUG_ENABLE == 1
    Serial.println(GYRO_ERROR);
  #endif
}

void loop() {

  // put your main code here, to run repeatedly:
  if(millis() - timeCur > 10) {
    timePrev = timeCur;
    timeCur = millis();

    // Measure Gyro value
    yawAngularSpeed = ((double)readMPU()-GYRO_ERROR) / 32.8;
    yawAngle += (yawAngularSpeed * (timeCur - timePrev) / 1000.0);
        // Put angle between -180 and 180
    while (yawAngle <= -180) yawAngle += 360; 
    while (yawAngle > 180)   yawAngle -= 360;
    
    total = total - readings[readIndex]; 
    readings[readIndex] = yawAngularSpeed;
    total = total + readings[readIndex];
    readIndex = readIndex + 1;
    if (readIndex >= numReadings){
      readIndex = 0;
    rollingAvg = total / numReadings; 
    }
  }
      Serial.print(yawAngle);
      Serial.print(" ");
      Serial.println(rollingAvg);
}
