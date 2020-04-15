#include <QTRSensors.h>

//Code to attempt to drive robot with both IR sensor and ultrasonic sensor
//Bot runs at max speed unless object is detected, when object is detect bot stops
//If bot runs off course appropriate motor is reduced to get back on track

#define KP 0.2
#define KD 5

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
const int trigPin = 6;
const int echoPin = 5;

float distance = 0;            //variable to store the distance measured by the distance sensor

int rightMP = 0;               //variable for motor speed
int leftMP = 0; 
int mP = 0;
int mPdes = 0; 
int Rec = 0;                    // variable for bluetooth communication
float Kp = 0.1;
float Kd = 1;     
int lastLineErr = 0;     

void setup()
{
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(13, OUTPUT);

  pinMode(trigPin, OUTPUT);       //this pin will send ultrasonic pulses out from the distance sensor
  pinMode(echoPin, INPUT);        //this pin will sense when the pulses reflect back to the distance sensor

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
  // Check for collisions
  distance = getDistance();

  // If a collision is detected, stop. Else, maintain speed
  if(distance < 10)
  {
    mP = 0;
  }
  else
  {
    mP = mPdes;
  }

  // Check for message from user
  // If there is a message, adjust speed accordingly
  if (Serial.available()>0)
  {
    Rec = Serial.read();
    if(Rec == 's')
    {
      mPdes = 0;
      mP = mPdes;
    }
    if(Rec == 'u')
    {
      mPdes += 51;
      mP = mPdes;
    }
    if(Rec == 'd')
    {
      mPdes -= 51;
      mP = mPdes;
    }
  }

  // Get position from IR sensor
  uint16_t position = qtr.readLineBlack(sensorValues);
  // Below is for debugging IR sensor
  /*for (uint8_t i = 0; i < SensorCount; i++)
    {
      Serial.print(sensorValues[i]);
      Serial.print('\t');
    }
    Serial.println(position);*/
    
  //Use position to determine appropriate compensation
  //rightMP = steerRight(mP,position);
  //leftMP = steerLeft(mP,position);

  //Control position
  int error = position - 2500;
  int speed = KP*error + KD*(error - lastLineErr);
  lastLineErr = error;
  if (speed > 255)
  {
    speed = 255;
  }
  else if (speed < 0)
  {
    speed = 0;
  }
  int leftMP = mP + speed;
  int rightMP = mP - speed;
    
  //Apply action to motors
  rightMotor(-rightMP);
  leftMotor(leftMP);
}

/**********************************************************************************/
int steerRight(int mP, uint16_t position)
{
  if (position < 2200)// was 1400
  {
    rightMP = 0;
  }
  /*if (position > 1400 && position < 1800)
  {
    rightMP = round((1/3)*mP); 
  }
  if (position > 1800 && position < 2200)
  {
    rightMP = round((1/3)*mP);
  }*/
  if (position >= 2200)
  {
    rightMP = mP;
  }
  return rightMP;
}

/*********************************************************************************/
int steerLeft(int Mp, uint16_t position)
{
  if (position  <= 2600)
  {
    leftMP = mP;
  }
  /*if (position > 2400 && position < 2850)
  {
    leftMP = round((1/3)*mP);;
  }
  if (position > 2850 && position < 3300)
  {
    leftMP = round((1/3)*mP); 
  }*/
  if (position > 2600) // was 3300
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
float getDistance()
{
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calcualtedDistance;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10ms long
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor

  calcualtedDistance = echoTime / 148.0;  //calculate the distance of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  return calcualtedDistance;              //send back the distance that was calculated
}
