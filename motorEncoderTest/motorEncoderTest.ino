#include <PinChangeInt.h>
//#include <SoftwareSerial.h>

#define pwmA 3
#define dirA 12
#define pwmB 11
#define dirB 13
#define deadbandA 30
#define deadbandB 30
#define A 1
#define B -1
#define pushButton 2
#define leftEncoderA 6
#define leftEncoderB 7

int counter = 0;
int aState;
int aLastState;

void setup() {
  // put your setup code here, to run once:
  pinMode(pwmA, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(leftEncoderA, INPUT);
  pinMode(leftEncoderB, INPUT);

  digitalWrite(dirA, HIGH);
  analogWrite(pwmA, 125);
  Serial.begin(9600);

  aLastState = digitalRead(leftEncoderA);
  PCintPort::attachInterrupt(leftEncoderB, runEncoder1, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
    Serial.print("Position: ");
    Serial.println(counter);
}
/*
void leftISR() {
  leftCount++;
}*/

void runEncoder1() {
  if(digitalRead(leftEncoderA) == digitalRead(leftEncoderB)){
    counter++;
  } else {
    counter--;
  }
}
