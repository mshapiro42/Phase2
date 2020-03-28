#include <QTRSensors.h>

//Code to attempt to drive robot with both IR sensor and ultrasonic sensor
//Bot runs at max speed unless object is detected, when object is detect bot stops
//If bot runs off course appropriate motor is reduced to get back on track

QTRSensors qtr;

//float getDistance();
void rightMotor(int);
void leftMotor(int);
int steerRight(int, uint16_t);
int steerLeft(int, uint16_t);

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];


//the right motor will be controlled by the motor A pins on the motor driver
const int AIN1 = 6;           //control pin 1 on the motor driver for the right motor
const int AIN2 = 12;            //control pin 2 on the motor driver for the right motor
const int PWMA = 11;            //speed control pin on the motor driver for the right motor

//the left motor will be controlled by the motor B pins on the motor driver
const int PWMB = 10;           //speed control pin on the motor driver for the left motor
const int BIN2 = 9;           //control pin 2 on the motor driver for the left motor
const int BIN1 = 8;           //control pin 1 on the motor driver for the left motor


//distance variables
//const int trigPin = 6;
//const int echoPin = 5;

int switchPin = 7;             //switch to turn the robot on and off

//float distance = 0;            //variable to store the distance measured by the distance sensor

int rightMP = 0;               //variable for motor speed
int leftMP = 0; 
int mP = 0; 
int Rec = 0;                    // variable for bluetooth communication            

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);

  //pinMode(trigPin, OUTPUT);       //this pin will send ultrasonic pulses out from the distance sensor
  //pinMode(echoPin, INPUT);        //this pin will sense when the pulses reflect back to the distance sensor

// Switch needs to be used when I'm using serial with computer instead of Bluetooth to read/debug IR sensor code
  pinMode(switchPin, INPUT_PULLUP);   //set this as a pullup to sense whether the switch is flipped


  //set the motor contro pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  digitalWrite(13, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
//   0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
//   * 10 reads per calibrate() call = ~24 ms per calibrate() call.
//   Call calibrate() 400 times to make calibration take about 10 seconds.
for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(13, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  /*for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();*/
  delay(1000);
  
}

void loop()
{
  if (Serial.available()>0)
  {
    Rec = Serial.read();
    if(Rec == 's')
    {
      mP = 0;
    }
    if(Rec == 'u')
    {
      mP += 51;
    }
    if(Rec == 'd')
    {
      mP -= 51;
    }
    //uint16_t position = qtr.readLineBlack(sensorValues);
    //rightMP = steerRight(mP,position);
    //leftMP = steerLeft(mP,position);
    rightMP = mP; // This is only for testing bluetooth without using IR sensor, with IR sensor above two lines should be used
    leftMP = mP;
    rightMotor(-rightMP);
    leftMotor(leftMP);
  }
    
  // Distance code below has been dismantled a bit, needs to be integrated with above when ready
  //DETECT THE DISTANCE READ BY THE DISTANCE SENSOR
  //  distance = getDistance();
  //                       
  //if the switch is off then stop (this isn't necessary with bluetooth controlling stopping)
  //stop the motors
  //DETECT THE DISTANCE READ BY THE DISTANCE SENSOR
  //  distance = getDistance();
  //if the on switch is flipped (this isn't necessary with bluetooth controlling going)
  //
  //    if(distance < 10){                //if an object is detected
  //
  //      //stop for a moment
  //      rightMotor(0);
  //      leftMotor(0);
  //    }else{                         //if no obstacle is detected drive forward
  //
  //      rightMotor(-rightMP);
  //      leftMotor(leftMP);
  //    }
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)

// Below is for debugging IR sensor
  /*for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);*/
}

/**********************************************************************************/
int steerRight(int mP, uint16_t position)
{
  if (position < 1400)
  {
    rightMP = 0;
  }
  if (position > 1400 && position < 1800)
  {
    rightMP = round((1/3)*mP); 
  }
  if (position > 1800 && position < 2200)
  {
    rightMP = round((1/3)*mP);
  }
  if (position > 2200)
  {
    rightMP = mP;
  }
  return rightMP;
}

/*********************************************************************************/
int steerLeft(int Mp, uint16_t position)
{
  if (position  < 2400)
  {
    leftMP = mP;
  }
  if (position > 2400 && position < 2850)
  {
    leftMP = round((1/3)*mP);;
  }
  if (position > 2850 && position < 3300)
  {
    leftMP = round((1/3)*mP); 
  }
  if (position > 3300)
  {
    leftMP = 0;
  }
  return leftMP;
}

/********************************************************************************/
void rightMotor(int motorSpeed)                       //function for driving the right motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(AIN1, HIGH);                         //set pin 1 to high
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backwar (negative speed)
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(AIN1, LOW);                          //set pin 1 to low
    digitalWrite(AIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMA, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
void leftMotor(int motorSpeed)                        //function for driving the left motor
{
  if (motorSpeed > 0)                                 //if the motor should drive forward (positive speed)
  {
    digitalWrite(BIN1, HIGH);                         //set pin 1 to high
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  else if (motorSpeed < 0)                            //if the motor should drive backwar (negative speed)
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, HIGH);                         //set pin 2 to high
  }
  else                                                //if the motor should stop
  {
    digitalWrite(BIN1, LOW);                          //set pin 1 to low
    digitalWrite(BIN2, LOW);                          //set pin 2 to low
  }
  analogWrite(PWMB, abs(motorSpeed));                 //now that the motor direction is set, drive it at the entered speed
}

/********************************************************************************/
//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
//float getDistance()
//{
//  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
//  float calcualtedDistance;         //variable to store the distance calculated from the echo time
//
//  //send out an ultrasonic pulse that's 10ms long
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10); 
//  digitalWrite(trigPin, LOW);
//
//  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
//                                          //pulse to bounce back to the sensor
//
//  calcualtedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
//
//  return calcualtedDistance;              //send back the distance that was calculated
//}
