/*****************************************************************
  Serial1_Remote_Control.ino
  Write your Arduino's pins (analog or digital) or read from those
  pins (analog or digital) using a remote Serial1.
  Jim Lindblom @ SparkFun Electronics
  Original Creation Date: May 7, 2014

  This sketch requires an Serial1, Serial1 Shield and another Serial1 tied to
  your computer (via a USB Explorer). You can use XCTU's console, or
  another serial terminal program (even the serial monitor!), to send
  commands to the Arduino.

  Example usage (send these commands from your computer terminal):
    w#nnn - analog WRITE pin # to nnn
      e.g. w6088 - write pin 6 to 88
    d#v   - digital WRITE pin # to v
      e.g. ddh - Write pin 13 High
    r#    - digital READ digital pin #
      e.g. r3 - Digital read pin 3
    a#    - analog READ analog pin #
      e.g. a0 - Read analog pin 0

    - Use hex values for pins 10-13
    - Upper or lowercase works
    - Use 0, l, or L to write LOW
    - Use 1, h, or H to write HIGH

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

  This code is beerware; if you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful, please
  buy us a round!

  Distributed as-is; no warranty is given.
*****************************************************************/
// SoftwareSerial is used to communicate with the Serial1
//#include <SoftwareSerial.h>

#define pwmA 3
#define dirA 12
#define pwmB 11
#define dirB 13
#define A 0
#define B 1
#define deadbandA 30
#define deadbandB 30
#define pushButton 2
#define TURN_TIME 300
#define GAIN .87

int8_t left_read, right_read;
int left_value, right_value, pushValue;
int systemState;
//SoftwareSerial Serial1(4, 5); // Arduino RX, TX (Serial1 Dout, Din)
int setA = 0;
int setB = 0;
int tempA = 0;
int tempB = 0;
byte cmd;
void setup()
{
  // Initialize Serial1 Software Serial port. Make sure the baud
  // rate matches your Serial1 setting (9600 is default).
  Serial1.begin(9600);
  Serial1.println("Initializing");
  Serial.begin(9600);
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  systemState = false;
  Serial1.println("Ready");
}

void loop()
{

  // In loop() we continously check to see if a command has been
  //  received.
  if (Serial1.available()) {
    cmd = Serial1.read();
  }
  if (cmd == 'p') {
    systemState = !systemState;
    if (systemState) {
      Serial1.print("Power on");
    }
    else {
      Serial1.print("Power off");
      setA = 0;
      setB = 0;
      Stop();
    }
  }
  if (systemState && cmd != 'p') {
    switch (cmd) {
      case 's':
        setA = 100;
        setB = 100;
        Forward(100);
        Serial1.print("Slow");
        break;
      case 'f':
        setA = 255;
        setB = 255;
        Forward(255);
        Serial1.print("Fast");
        break;
      case 'r':
        Turn(-1, setA, setB);
        Serial1.print("Right Turn");
        break;
      case 'l':
        Turn(1, setA, setB);
        Serial1.print("Left Turn");
        break;
    }
  }
  delay(100);
}

///////////////////////////////////
void Turn(int dir, int a, int b) {
  motor(A, -dir * 255);
  motor(B, dir * 255);
  delay(TURN_TIME);
  motor(A, a);
  motor(B, b);
}
///////////////////////////////////
void Backward(int s) {
  motor(A, -s);
  motor(B, -s*GAIN);
}
///////////////////////////////////
void Forward(int s) {
  motor(A, s);
  motor(B, s*GAIN);
}
///////////////////////////////////
void Stop() {
  motor(A, 0);
  motor(B, 0);
}
///////////////////////////////////
void motor(int m, int pwm) {
  int dirPin, pwmPin, deadband;
  switch (m) {
    case A:
      digitalWrite(dirA, (pwm >= 0));
      if (abs(pwm) >=  deadband) {
        analogWrite(pwmA, abs(pwm));
      } else {
        analogWrite(pwmA, 0);
      }
      break;
    case B:
      pwm *= GAIN;
      digitalWrite(dirB, (pwm >= 0));
      if (abs(pwm) >=  deadband) {
        analogWrite(pwmB, abs(pwm));
      } else {
        analogWrite(pwmB, 0);
      }
      break;
  }

}
