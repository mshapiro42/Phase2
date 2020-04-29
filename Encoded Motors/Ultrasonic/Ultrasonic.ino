

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
#include <QTRSensors.h>

// Contstants
#define TRIG_PIN 9
#define ECHO_PIN 8
#define TURN_TIME 150
#define sensorCount 6
#define kp_v 50
#define kd_v 25
#define ki_v 0
#define kp_p 0.03
#define kd_p 0
#define ki_p 0
#define deadbandA 100
#define deadbandB 100
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
QTRSensors qtr;

int systemState;
byte cmd2;

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;
float desVelL = 0;
float desVelR = 0;
float appVelL = 0;
float appVelR = 0;
float dir = 0;


void setup()
{
  // Initialize Serial1 Software Serial port. Make sure the baud
  // rate matches your Serial1 setting (9600 is default).
  pinMode(leftEncoderA, INPUT);
  pinMode(leftEncoderB, INPUT);
  pinMode(rightEncoderA, INPUT);
  pinMode(rightEncoderB, INPUT);
  pinMode(7, OUTPUT);
  analogWrite(7,50);
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial1.println("Initializing");
  Serial.println("Initializing");
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A0, A1, A2, A3, A4, A5
  }, sensorCount);
  qtr.setEmitterPin(6);
  
  qik.init();
  delay(1000);

  digitalWrite(13, HIGH);  
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(13, LOW);

  qik.setSpeeds(0, 0);
  attachInterrupt(digitalPinToInterrupt(leftEncoderB), leftEnc, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderB), rightEnc, RISING);

  systemState = false;
  Serial1.println("Ready");
  Serial.println("Ready");
}
void loop()
{

  uint16_t sensorValues[sensorCount];
  // In loop() we continously check to see if a command has been
  //  received.
  if (getDistance() < 10) {   //If object is within 10 inches
    appVelL = 0;              //Stop device
    appVelR = 0;
  }
  else {
    appVelL = desVelL;        //Otherwise, maintain desired speed
    appVelR = desVelR;
  }
  if (Serial1.available()) {
    readBT();
  }
  Serial1.println(getPosition(&sensorValues[0]));
  Serial.println(getPosition(&sensorValues[0]));
  setVel();
  delay(100);
}

///////////////////////////////////
void setVel() {
  float curVelL;
  float curVelR;
  int sensorValues[sensorCount];
  static int setA = 0;
  static int setB = 0;
  curVelL = getVelL();
  curVelR = getVelR();
  float errP = 2500 - (int)getPosition(&sensorValues[0]);
  float errL = appVelL - curVelL;
  float errR = appVelR - curVelR;

  while ((abs(errP) > 500 || abs(errL) > .05 || abs(errR) > .05) && systemState) {
    setA += computeUV(errL) + computeUP(errP);
    setB += computeUV(errR) - computeUP(errP);
    setA = applyLims(0, setA);
    setB = applyLims(1, setB);
    qik.setSpeeds(setA, setB);
    delay(100);
    if (Serial1.available()) {
      readBT();
    }
    curVelL = getVelL();
    curVelR = getVelR();
    errL = appVelL - curVelL;
    errR = appVelR - curVelR;
    errP = 2500 - (int)getPosition(&sensorValues[0]);
    Serial1.print(setA);
    Serial1.print(" ");
    Serial1.println(setB);
  }
  if (systemState) qik.setSpeeds(setA, setB);
}
///////////////////////////////////
uint16_t getPosition(int* array_ptr) {
  return qtr.readLineBlack(array_ptr);
}
///////////////////////////////////
int applyLims(int motor, int pwm) {
  int deadband;
  int dir = pwm / abs(pwm);
  pwm = abs(pwm);
  switch (motor) {
    case 0:
      deadband = deadbandA;
      break;
    case 1:
      deadband = deadbandB;
      break;
  }
  if (pwm < deadband) {
    pwm = deadband;
  } else if (pwm > 255) {
    pwm = 255;
  }
  return dir * pwm;
}
///////////////////////////////////
float computeUV(float errV) {
  volatile float errL = 0;
  volatile float errSum = 0;
  volatile unsigned long ti = 0;
  volatile unsigned long ti_1 = 0;
  float dE = errV - errL;
  errSum += dE;
  ti_1 = millis() / 1000;
  int dt = ti_1 - ti;

  float u = kp_v * errV + kd_v * (dE / dt) + ki_p * errSum;


  errL = errV;
  ti = ti_1;
  return u;
}
///////////////////////////////////
float computeUP(float errP) {
  volatile float errL = 0;
  volatile float errSum = 0;
  volatile unsigned long ti = 0;
  volatile unsigned long ti_1 = 0;
  float dE = errP - errL;
  errSum += dE;
  ti_1 = millis() / 1000;
  int dt = ti_1 - ti;

  float u = kp_p * errP + kd_p * (dE / dt) + ki_p * errSum;

  errL = errP;
  ti = ti_1;
  return u;
}

///////////////////////////////////
float readBT() {
  cmd2 = Serial1.read();

  switch (cmd2) {
    case 's':
      desVelL = 0;
      desVelR = 0;
      systemState = false;
      Serial1.println("Stop");
      break;
    case 'g':
      systemState = true;
      Serial1.println("Power On");
      break;
    case 'u':
      desVelL += 0.25;
      desVelR += 0.25;
      break;
    case 'd':
      desVelL -= 0.25;
      desVelR -= 0.25;
      break;
  }
  if (desVelL > 5) desVelL = 5;
  else if (desVelL < -5) desVelL = -5;
  if (desVelR > 5) desVelR = 5;
  else if (desVelR < -5) desVelR = -5;
}
/*
  ///////////////////////////////////
  void Turn(int dir, int setA, int setB) {
  qik.setSpeeds(dir * 255, -GAIN * dir * 255);
  delay(TURN_TIME * abs(dir) / 90);
  qik.setSpeeds(setA, setB);
  }*/
///////////////////////////////////
float getVelL() {
  int a1 = leftCount;
  delay(10);
  int b1 = leftCount;
  float curVelL = ((b1 - a1) / .01) * CPS_TO_MPH;
  return curVelL;
}
///////////////////////////////////
float getVelR() {
  int a2 = rightCount;
  delay(10);
  int b2 = rightCount;
  float curVelR = ((b2 - a2) / .01) * CPS_TO_MPH;
  return curVelR;
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
///////////////////////////////////
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calcualtedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10); 
  digitalWrite(TRIG_PIN, LOW);

  echoTime = pulseIn(ECHO_PIN, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor

  calcualtedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  return calcualtedDistance;              //send back the distance that was calculated
}
