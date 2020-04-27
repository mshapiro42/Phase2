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

#define TURN_TIME 150
#define GAIN .85
#define COUNTS_PER_REV 979.62
#define IN_PER_REV 10.039
#define IN_PER_FT 12
#define FT_PER_MILE 5280
#define IN_PER_MILE IN_PER_FT*FT_PER_MILE
#define SEC_PER_HR 3600
#define CPS_TO_MPH IN_PER_REV*SEC_PER_HR/(COUNTS_PER_REV*IN_PER_MILE)
#define leftEncoderB 2
#define rightEncoderB 3
#define leftEncoderA 4
#define rightEncoderA 5
//#define GAIN .87
PololuQik2s9v1 qik(10,11,12);
int systemState;
int setA = 0;
int setB = 0;
byte cmd2;

volatile unsigned long leftCount = 0;
volatile unsigned long rightCount = 0;
float velocityL = 0;
float velocityR = 0;
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


  attachInterrupt(digitalPinToInterrupt(leftEncoderB),leftEnc,RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderB),rightEnc,RISING);

  systemState = false;
  Serial1.println("Ready");
  Serial.println("Ready");
}
void loop()
{

  // In loop() we continously check to see if a command has been
  //  received.
  if (Serial1.available()) {
    readBT();
  }
  if (systemState){
    qik.setSpeeds(setA, setB);
    getVel();
    Serial1.print(setA);
    Serial1.print(" ");
    Serial1.println(setB);
  } else{
    qik.setSpeeds(0, 0);
  }
  delay(100);
}


void readBT() {
  cmd2 = Serial1.read();
  
  if (cmd2 == 'p') {
    systemState = !systemState;
    if (systemState) {
      //Serial1.println("Power on");
    }
    else {
      //Serial1.println("Power off");      
      setA = 0;
      setB = 0;
    }
  }
  if (systemState && cmd2 != 'p') {
    switch (cmd2) {
      case 's':
        setA = 100;
        setB = GAIN*100;
        //Serial1.println("Slow");
        break;
      case 'i':
        setA = -100;
        setB = -GAIN*100;
        //Serial1.println("Reverse");
        break;
      case 'r':
        Turn(-1, setA, setB);
        //Serial1.println("Right Turn");
        break;
      case 'l':
        Turn(1, setA, setB);
        //Serial1.println("Left Turn");
        break;         
    }
  }
}
///////////////////////////////////
void Turn(int dir, int setA, int setB) {
  qik.setSpeeds(dir*255,-GAIN*dir*255);
  delay(TURN_TIME);
  qik.setSpeeds(setA, setB);
}
///////////////////////////////////
void getVel() {
  int a1 = leftCount;
  int a2 = rightCount;
  delay(10);
  int b1 = leftCount;
  int b2 = rightCount;
  velocityL = ((b1-a1)/.01)*CPS_TO_MPH;
  velocityR = ((b2-a2)/.01)*CPS_TO_MPH;
}
///////////////////////////////////
void leftEnc(){
  if (digitalRead(leftEncoderA) == digitalRead(leftEncoderB)){
    leftCount++;
  } else {
    leftCount--;
  }
}
///////////////////////////////////
void rightEnc(){
  if (digitalRead(rightEncoderA) == digitalRead(rightEncoderB)){
    rightCount++;
  } else {
    rightCount--;
  }
}
