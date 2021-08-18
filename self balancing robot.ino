#include <MPU6050_tockn.h>

MPU6050 mpu6050(Wire);

float Accy, Accz;
float gyrox;
float accangle;
int leftpwm = 3;
int rightpwm = 6;
int left = 4;
int right = 8;
float maximum=0;
float minimum=1023;
int kp = 60;
int kd = 20;
int ki = 80;
float setpt;
float ip, op;
int currentime, elapsedtime, prevtime;
float error, integerror, differror, preverror;
float k;
float gyroang = 0;
float currentang = 0;
float prevang = 0;
float motorip = 0;
float gyrorate;
int looptime, curtime, pretime;
float sampleTime = 0.005;

void wheels (int leftwheel, int rightwheel){
    if (leftwheel >= 0){
        analogWrite(leftpwm, leftwheel);//back
        digitalWrite(left, LOW);
        //Serial.println("front");
        
    }
    else{
        analogWrite(leftpwm, 1000+leftwheel);//front
        digitalWrite(left, HIGH);
        //Serial.println("back");
    }
    if (rightwheel >= 0){
        analogWrite(rightpwm, rightwheel);
        digitalWrite(right, LOW);
        
    } 
    else{
        analogWrite(rightpwm, 1000+rightwheel);
        digitalWrite(right, HIGH);
    }
    
}
  
void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}


void setup(){
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();
    mpu6050.setGyroOffsets(2.20, 1.67, 0.62);
    pinMode(leftpwm, OUTPUT);
    pinMode(left, OUTPUT);
    pinMode(rightpwm, OUTPUT);
    pinMode(right, OUTPUT);
    setpt=-2.5;
    init_PID();
}

void loop(){
    
    mpu6050.update();
    Accy = mpu6050.getAccY();
    Accz = mpu6050.getAccZ();
    gyrox = mpu6050.getGyroX();
    
    motorip = constrain(motorip,-1000,1000);
    Serial.println(motorip);
    wheels(motorip,motorip);
    
}

ISR(TIMER1_COMPA_vect){
  accangle = atan2(Accy,Accz)*RAD_TO_DEG;

    
    
  gyrorate = map(gyrox,-32768, 32767, -250, 250);
  gyroang = (float)gyrorate*sampleTime;

  currentang = 0.9934*(prevang+gyroang)+0.0066*accangle;
    
  //Serial.println(currentang);
    
        
  error = currentang - setpt;
  integerror += error;
  integerror = constrain(integerror,-300,300);
  differror = currentang-prevang;

  k = (kp*error) + ((kd*integerror)*sampleTime) - ((ki*differror)/sampleTime);

  prevtime = currentime;
  
  motorip = k;
  prevang = currentang;
}