#include <PinChangeInt.h>
//#include <SoftwareSerial.h>

#define pwmA 3
#define dirA 12
#define pwmB 11
#define dirB 13
#define deadbandA 0
#define deadbandB 30
#define A 1
#define B -1
#define pushButton 2
#define leftEncoderA 6
#define leftEncoderB 7

#define kp_left 1

#define COUNTS_PER_REV 48
#define DIST_PER_REV .1 // cm

int32_t leftCounter = 0;
uint16_t setA;
int systemState = 0;
int sampleTime = 200; // ms

void setup() {
  // put your setup code here, to run once:
  pinMode(pwmA, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(leftEncoderA, INPUT);
  pinMode(leftEncoderB, INPUT);
  pinMode(pushButton, INPUT_PULLUP);

  Serial.begin(9600);

  PCintPort::attachInterrupt(leftEncoderB, runEncoder1, RISING);
  PCintPort::attachInterrupt(pushButton, state, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("waiting for button");

  while(systemState){  
    digitalWrite(dirA, LOW);
    velTest();
    
  }
}

void velTest() {
  unsigned long timeInit;
  for(int set = 135; set <= 255; set += 5){    
    timeInit = millis();
    analogWrite(pwmA, set);
    while(millis()-timeInit < 1000){    
      int meas1 = leftCounter;
      Serial.print(meas1/1000);
      delay(500);
      int meas2  = leftCounter;
      Serial.print(" ");
      Serial.print(meas2/1000);
      Serial.print(" ");
      Serial.println(((meas2-meas1)/.5)/COUNTS_PER_REV);
    }
  }
   for(int set = 255; set >= 135; set -= 5){    
    timeInit = millis();
    analogWrite(pwmA, set);
    while(millis()-timeInit < 1000){    
      int meas1 = leftCounter;
      Serial.print(meas1/1000);
      delay(500);
      int meas2  = leftCounter;
      Serial.print(" ");
      Serial.print(meas2/1000);
      Serial.print(" ");
      Serial.println(((meas2-meas1)/.5)/COUNTS_PER_REV);
    }
  }
}

void drive(int velDesired) {
  int cmdLeft, errorLeft, curVel;
  setA = 0;
  leftCounter = 0;
  errorLeft = velDesired;
  
  while (errorLeft > 0 && systemState) {
    setA += computeCommand(A, errorLeft);
    motor(A, setA);
    curVel = computeVel(A);
    errorLeft = velDesired - curVel;
    Serial.print("Vel Des: ");
    Serial.print(velDesired);
    Serial.print(" Cur Vel: ");
    Serial.print(curVel);
    Serial.print(" CMD Left: ");
    Serial.println(setA);
  }
  motor(A, 0);
}

void motorStartup(int m, int dir) {
  motor(m, dir * 255);
  delay(100);
}

int computeCommand(int m,  int e) {
  int kp;
  int set = 0;
  int dir = e / abs(e);
  switch(m) {
    case A:
      kp = kp_left;
      break;
  }
  set = kp*e;
  if (abs(set) > 255) {
    set = dir * 255;
  }
  return (set);
}

int computeVel(int m){ // deltacounts/sec
  int meas1, meas2;
  float vel;
  switch(m){
    case A:
      meas1 = leftCounter;
      delay(sampleTime);
      meas2 = leftCounter;
      break;
  }
  Serial.print("delta_count: ");
  Serial.print(meas2-meas1);
  Serial.print(" vel: ");
  Serial.print((meas2-meas1)/sampleTime);
  Serial.println(" cnt/ms");
  delay(1000);
  return ((meas2 - meas1)/sampleTime)/COUNTS_PER_REV*DIST_PER_REV;
}
void motor(int m, int pwm) {
  int pwmPin, dirPin, deadband;
  switch (m) {
    case A:
      dirPin = dirA;
      pwmPin = pwmA;
      deadband = deadbandA;
      break;
    case B:
      dirPin = dirB;
      pwmPin = pwmB;
      deadband = deadbandB;
      break;
  }

  digitalWrite(pwmPin, (pwm >= 0));
  if (abs(pwm) > 255) {
    analogWrite(pwmPin, 255);
  } else if (pwm == 0 ) {
    analogWrite(pwmPin, 0);
  }
}

void runEncoder1() {
  if (digitalRead(leftEncoderA) == digitalRead(leftEncoderB)) {
    leftCounter++;
  } else {
    leftCounter--;
  }
}


void state () {
  systemState = !systemState;
  if(!systemState){
    motor(A,0);
  }
  delay(100);
}
