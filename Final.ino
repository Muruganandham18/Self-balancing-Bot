#include <MPU6050.h>
#include <I2Cdev.h>

#include <Wire.h>
#define targetAngle 1.2
#define Kp  100
#define Kd  1.5
#define Ki  2
long accelX, accelY, accelZ;
long t;
float gForceX, gForceY, gForceZ;
MPU6050 accelgyro(0x68);
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

double sampleTime;
double accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
float gyroRate;
double motorPower;
#define leftMotorPWMPin   6
#define rightMotorPWMPin  5


void forwardright(void)
{ 
 digitalWrite(8,LOW);
 digitalWrite(12,HIGH);
}

void forwardleft(void)
{
 digitalWrite(2,LOW);
 digitalWrite(4,HIGH); 

  
}

void backwardleft(void)
{
  digitalWrite(2,HIGH);
 digitalWrite(4,LOW); 
}

void backwardright(void)
{ 
 digitalWrite(8,HIGH);
 digitalWrite(12,LOW);
}

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    forwardleft();
  }
  else {
    
    analogWrite(leftMotorPWMPin, abs(leftMotorSpeed));
    backwardleft();
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    forwardright();
    
  }
  else {
    analogWrite(rightMotorPWMPin, abs(rightMotorSpeed));
    backwardright();
  }
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  accelgyro.initialize();
  pinMode(8,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(4,OUTPUT);
accelgyro.setXAccelOffset(48);
accelgyro.setYAccelOffset(-192);
accelgyro.setZAccelOffset(1567);
accelgyro.setXGyroOffset(-29);
accelgyro.setYGyroOffset(110);
accelgyro.setZGyroOffset(-50);
 
}


void loop() {  
 
recordAccelRegisters();
recordGyroRegisters();
sampleTime = millis()-t;
 
   if(sampleTime>=5)
   {
  accAngle = atan2(accelY, accelZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  sampleTime=sampleTime*0.001;
  
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9*(prevAngle + gyroAngle) + 0.1*(accAngle);
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  Serial.println(currentAngle);  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  t=millis();
   }
   motorPower =  constrain(motorPower, -255, 255);
   //Serial.println(motorPower);
  setMotors(motorPower, motorPower);
 
  printData();
  delay(1);
}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
}

void printData() {
//  Serial.print("Gyro (deg)");
//  Serial.print(" X=");
//  Serial.print(rotX);
//  Serial.print(" Y=");
//  Serial.print(rotY);
//  Serial.print(" Z=");
//  Serial.print(rotZ);
//  Serial.print(" Accel (g)");
//  Serial.print(" X=");
//  Serial.print(gForceX);
//  Serial.print(" Y=");
//  Serial.print(gForceY);
//  Serial.print(" Z=");
//  Serial.println(gForceZ);
//  Serial.println(t);
}
