#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

float Accy, Accz;
float gyrox;
float accangle;
int leftpwm = 5;
int rightpwm = 7;
int left = 6;
int right = 8;

int kp = 2;
int kd = 5;
int ki = 6;
float setpt;
float ip, op;
int currentime, elapsedtime, prevtime;
float error, integerror, differror, preverror;
float k;
float gyroang = 0;
float currentang = 0;
float prevang = 0;
float motorip = 0;

void wheels (int leftwheel, int rightwheel){
    if (leftwheel >= 0){
        analogWrite(leftpwm, leftwheel);
        digitalWrite(left, LOW);
    else{
        analogWrite(leftpwm, 255+leftwheel);
        digitalWrite(left, LOW);
    }
    if (rightwheel >= 0){
        analogWrite(rightpwm, rightwheel);
        digitalWrite(right, LOW);
    else{
        analogWrite(rightpwm, 255+rightwheel);
        digitalWrite(right, LOW);
    }
    }
}

float pid(float ip){
    currentime = millis();
    elapsedtime = currentime - prevtime;
        
    error = setpt - ip;
    integerror += error*elapsedtime;
    differror = (error - preverror)/elapsedtime;

    k = (kp*error) + (kd*differror) + (ki*integerror);

    prevtime = currentime;
    preverror = error;

    return k;
}

void setup(){
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();
    mpu6050.setYAccelOffset(1593);
    mpu6050.setZAccelOffset(963);
    mpu6050.setXGyroOffset(40);
    setpt=0;
}

void loop(){
    curtime = millis();
    looptime = curtime - pretime;
    pretime = curtime;
    
    mpu6050.update();
    Accy = mpu6050.getAccY();
    Accz = mpu6050.getAccZ();
    gyrox = mpu6050.getGyroX();

    accangle = atan2(Accy,Accz)*RAD_TO_DEG;

    gyrorate = map(gyrox,-32768, 32767, -250, 250);
    gyroang = gyroang + gyrorate * (looptime/1000);

    currentang = 0.9934*(prevangle+gyroang)+0.0066*accangle;
    
    motorip = pid(currentang);
    motorip = constrain(motorip,-255,255);
    wheels(motorip,motorip);
    prevang = currentang;

}