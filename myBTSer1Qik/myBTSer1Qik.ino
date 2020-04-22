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

#define TURN_TIME 300
#define GAIN .87
PololuQik2s9v1 qik(10,11,12);
int left_value, right_value, pushValue;
int systemState;
//SoftwareSerial Serial1(4, 5); // Arduino RX, TX (Serial1 Dout, Din)
int setA = 190;
int setB = 0;
byte cmd2;
void setup()
{
  // Initialize Serial1 Software Serial port. Make sure the baud
  // rate matches your Serial1 setting (9600 is default).
  Serial1.begin(9600);
  Serial.begin(115200);
  Serial1.println("Initializing");
  Serial.println("Initializing");
  
  qik.init();
  delay(1000);
  qik.setM0Speed(setA);

  systemState = false;
  Serial1.println("Ready");
  Serial.println("Ready");
}
void loop()
{

  // In loop() we continously check to see if a command has been
  //  received.
  if (Serial1.available()) {
    cmd2 = Serial1.read();
  }
  if (cmd2 == 'p') {
    systemState = !systemState;
    if (systemState) {
      Serial1.println("Power on");
      Serial.println("Power on");
    }
    else {
      Serial1.println("Power off");
      Serial.println("Power off");
      
      Stop();
    }
  }
  if (systemState && cmd2 != 'p') {
    switch (cmd2) {
      case 's':
        setA = 100;
        setB = 100;
        Forward(100);
        Serial1.println("Slow");
        Serial.println("Slow");
        break;
      case 'f':
        setA = 255;
        setB = 255;
        Forward(255);
        qik.setSpeeds(255,255);
        Serial1.println("Fast");
        Serial.println("Fast");
        break;
      case 'r':
        Turn(-1, setA, setB);
        Serial1.println("Right Turn");
        Serial.println("Right Turn");
        break;
      case 'l':
        Turn(1, setA, setB);
        Serial1.println("Left Turn");
        Serial.println("Left Turn");
        break;
    }
  }
  delay(100);
}

///////////////////////////////////
void Turn(int dir, int a, int b) {
  //qik.setSpeeds(dir*255,-dir*255);
  
  qik.setM0Speed(dir*255);
  delay(TURN_TIME);
//  qik.setSpeeds(a,b);
   qik.setM0Speed(a);
//  motor(A, a);
//  motor(B, b);
}
///////////////////////////////////
void Backward(int s) {
  //qik.setSpeeds(-s,-s*GAIN);
  qik.setM0Speed(-s);
//  motor(A, -s);
//  motor(B, -s*GAIN);
}
///////////////////////////////////
void Forward(int s) {
//  qik.setSpeeds(s,s*GAIN);
  qik.setM0Speed(s); 
//  motor(A, s);
//  motor(B, s*GAIN);
}
///////////////////////////////////
void Stop() {
  //qik.setSpeeds(0,0);
  qik.setM0Speed(0);
//  motor(A, 0);
//  motor(B, 0);
}
///////////////////////////////////
//void motor(int m, int pwm) {
//  int dirPin, pwmPin, deadband;
//  switch (m) {
//    case A:
//      digitalWrite(dirA, (pwm >= 0));
//      if (abs(pwm) >=  deadband) {
//        analogWrite(pwmA, abs(pwm));
//      } else {
//        analogWrite(pwmA, 0);
//      }
//      break;
//    case B:
//      pwm *= GAIN;
//      digitalWrite(dirB, (pwm >= 0));
//      if (abs(pwm) >=  deadband) {
//        analogWrite(pwmB, abs(pwm));
//      } else {
//        analogWrite(pwmB, 0);
//      }
//      break;
//  }
//
//}
