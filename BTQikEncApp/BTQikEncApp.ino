/*****************************************************************
  Serial1_Remote_Control.ino
  Write your Arduino's pins (analog or digital) or read from those
  pins (analog or digital) using a remote Serial1.
  Jim Lindblom @ SparkFun Electronics
  Original Creation Date: May 7, 2014

  Hardware Hookup:
  The Arduino shield makes all of the connections you'll need
  between Arduino and Serial1. Make sure the SWITCH IS IN THE
  "DLINE" POSITION.

  Development environment specifics:
    IDE: Arduino 1.0.5
    Hardware Platform: SparkFun RedBoard
    Serial1 Shield & Serial1 Series 1 1mW (w/ whip antenna)
        Serial1 USB Explorer connected to computer with another
          Serial1 Series 1 1mW connected to that.
*****************************************************************/
// SoftwareSerial is used to communicate with the Serial1
#include <SoftwareSerial.h>
#include <PololuQik.h>
// Contstants
#define TURN_TIME 150
#define GAIN .95
#define kp 5
#define kd 1
#define ki 1

// Conversions
#define COUNTS_PER_REV 979.62
#define IN_PER_REV 10.039
#define IN_PER_FT 12
#define FT_PER_MILE 5280
#define IN_PER_MILE IN_PER_FT*FT_PER_MILE
#define SEC_PER_HR 3600
#define CPS_TO_MPH IN_PER_REV*SEC_PER_HR/(COUNTS_PER_REV*IN_PER_MILE)

//Pins
#define leftEncoderB 2
#define rightEncoderB 3
#define leftEncoderA 4
#define rightEncoderA 5

PololuQik2s9v1 qik(10, 11, 12);
int systemState;
volatile int setA = 0;
volatile  int setB = 0;
byte cmd2;

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;
float velocityL = 0;
float velocityR = 0;
float curVelL = 0;
float curVelR = 0;
float desVelL = 0;
float desVelR = 0;



void setup()
{
  // Initialize Serial1 Software Serial port. Make sure the baud
  // rate matches your Serial1 setting (9600 is default).
  pinMode(leftEncoderA, INPUT);
  pinMode(leftEncoderB, INPUT);
  pinMode(rightEncoderA, INPUT);
  pinMode(rightEncoderB, INPUT);

  Serial1.begin(9600);
  Serial1.println("Initializing");
  Serial.println("Initializing");

  qik.init();
  delay(1000);


  attachInterrupt(digitalPinToInterrupt(leftEncoderB), leftEnc, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderB), rightEnc, RISING);

  systemState = false;
  Serial1.println("Ready");
  Serial.println("Ready");
}
void loop()
{

  // In loop() we continously check to see if a command has been
  //  received.
  if (Serial1.available()) {
    desVel = readBT();
  }
  if (systemState) {
    getVelL();
    getVelR();
    setVel();
  } else {
    setSpeeds(0, 0);
  }
  delay(100);
}

///////////////////////////////////
void setVel(){
  int uL = computeU(curVelL, desVelL);
  int uR = computeU(curVelR, desVelR);
}
///////////////////////////////////
void computeU(int curVel, int desVel){
  volatile float errL = 0;
  volatile float errSum = 0;
  volatile unsigned long ti = 0;
  volatile unsigned long ti_1 = 0;
  float err = desVel - curVel;
  float dE = err - errL;
  errSum += dE;
  ti_1 = millis()/1000;
  dt = ti_1 - ti;

  int u = floor(kp * err + kd * dE/dt + ki*errSum);
  errL = err;
  ti = ti_1;
  return u;
  
}

///////////////////////////////////
float readBT() {
  cmd2 = Serial1.read();

  switch (cmd2) {
    case 's':
      systemState = false;
      Serial1.println("Stop");
      break;
    case 'g':
      systemState = true;
      Serial1.println("Power On");
      desVelL = 0.5;
      desVelR = 0.5;
      break;
    case 'u':
      desVelL += 0.5;
      desVelR += 0.5;
      Serial1.println(desVel);
      break;
    case 'd':
      desVelL -= 0.5;
      desVelR -= 0.5;
      Serial1.println(desVel);
      break;
  }
  if (desVelL > 5) desVelL = 5;
  else if (desVelL < -5) desVelL = -5;
  if (desVelR > 5) desVelR = 5;
  else if (desVelR < -5) desVelR = -5;
}

///////////////////////////////////
void Turn(int dir, int setA, int setB) {
  qik.setSpeeds(dir * 255, -GAIN * dir * 255);
  delay(TURN_TIME*abs(dir)/90);
  qik.setSpeeds(setA, setB);
}
///////////////////////////////////
void getVelL() {
  int a1 = leftCount;
  delay(10);
  int b1 = leftCount;
  curVelL = ((b1 - a1) / .01) * CPS_TO_MPH;
}
///////////////////////////////////
void getVelR() {
  int a2 = rightCount;
  delay(10);
  int b2 = rightCount;
  curVelR = ((b2 - a2) / .01) * CPS_TO_MPH;
}
///////////////////////////////////
void leftEnc() {
  if (digitalRead(leftEncoderA) == digitalRead(leftEncoderB)) {
    leftCount++;
  } else {
    leftCount--;
  }
}
///////////////////////////////////
void rightEnc() {
  if (digitalRead(rightEncoderA) == digitalRead(rightEncoderB)) {
    rightCount++;
  } else {
    rightCount--;
  }
}
