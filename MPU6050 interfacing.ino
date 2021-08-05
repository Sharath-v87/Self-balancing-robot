#include <I2Cdev.h>
#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);
float Accx, Accy, Accz;
float Gyrox, Gyroy, Gyroz;
float Accangx, Accangy;
float Gyroangx, Gyroangy;
float angx, angy, angz;
long timer = 0;

void setup(){
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop(){
  mpu6050.update();

  if(millis() - timer > 1000){
    Accx = mpu6050.getAccX();
    Accy = mpu6050.getAccY();
    Accz = mpu6050.getAccZ();
    Gyrox = mpu6050.getGyroX();
    Gyroy = mpu6050.getGyroY();
    Gyroz = mpu6050.getGyroZ();
    angx = mpu6050.getAngleX();
    angy = mpu6050.getAngleY();
    angz = mpu6050.getAngleZ();
    Serial.println(Accx);
    Serial.println(Accy);
    Serial.println(Accx);
    Serial.println(Gyrox);
    Serial.println(Gyroy);
    Serial.println(Gyroz);
    Serial.println(angx);
    Serial.println(angy);
    Serial.println(angz);
    timer = millis();
  }
  
}