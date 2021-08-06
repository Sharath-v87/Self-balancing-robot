int kp = 2;
int kd = 5;
int ki = 6;
int setpt;
int ip, op;
int currentime, elapsedtime, prevtime;
int error, integerror, differror, preverror;
int k;

int pid(int ip){
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
    setpt=0;
}

void loop(){
    ip = analogRead(A1);
    op = pid(ip);
    delay(100);
    analogWrite(6,op); 
}