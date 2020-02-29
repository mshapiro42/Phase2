#include <PinChangeInt.h>
//#include <SoftwareSerial.h>

#define pwmA 3
#define dirA 12
#define pwmB 11
#define dirB 13
//#define deadbandA 30
//#define deadbandB 30
#define A 1
#define B -1
#define pushButton 2
#define leftEncoderA 6
#define leftEncoderB 7
#define rightEncoderA 8
#define rightEncoderB 9

#define kp_left 1
#define kd_left 1
#define kp_right 1
#define kd_right 1

#define COUNTS_PER_REV 48
#define DIST_PER_REV 28 // cm

int leftCounter = 0;
int rightCounter = 0;
int setA, setB;
int systemState = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(pwmA, OUTPUT);
  pinMode(dirA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(leftEncoderA, INPUT);
  pinMode(leftEncoderB, INPUT);
  pinMode(rightEncoderA, INPUT);
  pinMode(rightEncoderB, INPUT);
  pinMode(pushButton, INPUT_PULLUP);

  Serial.begin(9600);

  PCintPort::attachInterrupt(leftEncoderB, runEncoder1, RISING);
  PCintPort::attachInterrupt(rightEncoderB, runEncoder2, RISING);
  PCintPort::attachInterrupt(pushButton, state, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!systemState){
    Serial.print(digitalRead(pushButton));
    Serial.print("\t");
      Serial.println(systemState);
  }
  Serial.println("Driving");
  drive(1000); //cm
  Serial.println("Done");
  delay(2000);

}

void drive(float dist) {
  int countsDesired, cmdLeft, errorLeft;
  int cmdRight, errorRight;
  int countLeft = 0;
  int countRight = 0;
  
  int power;

  countsDesired = (int) (dist / DIST_PER_REV) * COUNTS_PER_REV;
  leftCounter = 0;
  rightCounter = 0;
  errorLeft = 20;
  errorRight = 20;

  while (errorLeft > 0 && systemState) {
    setA = computeCommand(kp_left, errorLeft);
    setB = computeCommand(kp_left, errorRight);
    motor(A, setA);
    motor(B, setB);
    countLeft = leftCounter;
    errorLeft = countsDesired - countLeft;
    countRight = rightCounter;
    errorRight = countsDesired - countRight;
    Serial.print("Counts Des: ");
    Serial.print(countsDesired);
    Serial.print(" CMD Left: ");
    Serial.print(setA);
    Serial.print(" CMD Right: ");
    Serial.println(setB);
  }
  motor(A, 0);
  motor(B, 0);
}

void motorStartup(int m, int dir) {
  motor(m, dir * 255);
  delay(100);
}

int computeCommand(int gain, int e) {
  int setA = 0;
  int dir = e / abs(e);
  setA = gain * e;
  if (abs(setA) > 150) {
    setA = dir * 150;
  }
  return (setA);
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

void runEncoder2() {
  if (digitalRead(rightEncoderA) == digitalRead(rightEncoderB)) {
    rightCounter++;
  } else {
    rightCounter--;
  }
}

void state () {
  systemState = !systemState;
  if(!systemState){
    motor(A,0);
  }
  delay(100);
}
