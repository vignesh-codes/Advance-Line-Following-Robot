#include "motordriver_4wd.h"
#include <seeed_pwm.h>

#include <Wire.h>
#include "motordriver_4wd.h"
#include <seeed_pwm.h>

#define leftMotorBaseSpeed 15
#define rightMotorBaseSpeed 15

#define min_speed -50
#define max_speed 50

int leftMotorSpeed;  
int rightMotorSpeed;

unsigned char data[16];
unsigned int sensorData[8];
unsigned calData [8];

float Kp, Ki, Kd;
int i,n;
int previousTime;
float lastError;

char input;

void setup() {
  // put your setup code here, to run once:
  Wire.begin(); 
  Serial.begin (115200);
  MOTOR.init();

  PIDconstants();
  readSensorData();
  lastError = weightedAverage(); // set the first detected error as the reference error to calculate differential error when Hercules starts to move
  previousTime = millis(); // set 0 as the initial time before Hercules starts to move
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available () > 0)
  {
    input =  Serial.read (); // read message sent from Uno
    switch (input)
    {
      case 'a': // move forward if Uno sends an 'a'
        MOTOR.setSpeedDir (20, DIRF);
        break;

      case 'b': // turn left if Uno sends a 'b'
        MOTOR.setStop1();
        MOTOR.setStop2();
        MOTOR.setSpeedDir2 (60, DIRF);
        MOTOR.setSpeedDir1 (60, DIRR);
        break;

      case 'c': // turn right if Uno sends a 'c'
        MOTOR.setStop2();
        MOTOR.setStop1();
        MOTOR.setSpeedDir1 (60, DIRF);
        MOTOR.setSpeedDir2 (60, DIRR);
        break;

      case 'd': // stop if Uno sends a 'd'
      MOTOR.setStop1();
      MOTOR.setStop2();
      break;

      case 'e': // follow line with PID if Uno sends a 'e'
      PID ();

      default:
        break;
    }
  } 
 } 

void PID (void)
{
  readSensorData ();
  calibrateSensorData ();
  double error = weightedAverage();
  int output = PID(error);
  
  leftMotorSpeed = leftMotorBaseSpeed + output;     // Calculate the modified motor speed
  rightMotorSpeed = rightMotorBaseSpeed - output;

  int condition;
  
  if (leftMotorSpeed > 0 && rightMotorSpeed > 0)
  {
    leftMotorSpeed = constrain(leftMotorSpeed, 0, max_speed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, max_speed);

    condition = 1;
  }
  else if (leftMotorSpeed < 0 && rightMotorSpeed > 0) // set negative leftMotorSpeed to positive
  {
    leftMotorSpeed = constrain(leftMotorSpeed, min_speed, 0);
    leftMotorSpeed = leftMotorSpeed * -1;

    rightMotorSpeed = constrain(rightMotorSpeed, 0, max_speed);

    condition = 2;
  }
  else if (leftMotorSpeed > 0 && rightMotorSpeed < 0) // set negative rightMotorSpeed to positive 
  {
    rightMotorSpeed = constrain(rightMotorSpeed, min_speed, 0);
    rightMotorSpeed = rightMotorSpeed * -1;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, max_speed);

    condition = 3;    
  }

  switch (condition)
  {
    case 1:  // Move forward
    MOTOR.setSpeedDir1 (rightMotorSpeed, DIRF);
    MOTOR.setSpeedDir2 (leftMotorSpeed, DIRF); 
    break;

    case 2:  //Turn right
    MOTOR.setSpeedDir1 (rightMotorSpeed, DIRF);
    MOTOR.setSpeedDir2 (leftMotorSpeed, DIRR);
    break;

    case 3:  //Turn left
    MOTOR.setSpeedDir1 (rightMotorSpeed, DIRR);
    MOTOR.setSpeedDir2 (leftMotorSpeed, DIRF); 
    break;

    default:
    break;
  }
}

void PIDconstants (void)
{
  Kp = 20;
  Ki = 10;
  Kd = 100;
}

void readSensorData (void)
{
Wire.requestFrom(9, 16);    // request 16 bytes from slave device #9 which is the Line Follower Module
  while (Wire.available())   // slave may send less than requested
  {
    data[i] = Wire.read(); // receive a byte as character
    if (i < 15) // save the data into an array
    {
      i++;
    }
    else
    {
      i = 0;
    }
  }
  for(n=0;n<8;n++)
  {
    sensorData[n] = data[n*2]; //convert 16-bit array into 8-bit array
  }
}

void calibrateSensorData (void) // offset each value into a fixed average value so that each sensor
{                               // will give an approximately same value when sensing black and white line
  calData[0] = sensorData[0] + 14;
  calData[1] = sensorData[1] - 2;
  calData[2] = sensorData[2] + 7;
  calData[3] = sensorData[3] - 2;
  calData[4] = sensorData[4] - 50;
  calData[5] = sensorData[5] + 20;
  calData[6] = sensorData[6] - 4;
  calData[7] = sensorData[7] + 16;
}

double weightedAverage (void) 
{
  double weightedSum;
  double dataSum;
  double calibratedValue;
  double weightage[8] = {4.41,3.15,1.89,0.65,-0.65,-1.89,-3.15,-4.41}; // weightage fixed according to the distance between the midpoint and the sensor
  weightedSum = 0;                                                    
  dataSum = 0;
  for (n=0;n<8;n++) 
  {
    weightedSum += calData[n] * weightage[n];
  }
  for (n=0;n<8;n++)
  {
    dataSum += sensorData[n]; 
  }
  double weightedAverage; 
  weightedAverage = (weightedSum/dataSum);
  calibratedValue = (weightedAverage + 0.03 ) * 10; // offset weighted average value to 0 when the black line is at the middle
  return constrain (calibratedValue, -2.5, 2.5); // to prevent anomaly from sensor 7 from disrupting the symmetry of error
}

double PID (double error)
{
  double errorSum;
  double differentialError;  
 
  errorSum += error; // calculatr sum of error
  
  float currentError = weightedAverage();
  int currentTime = millis();
  differentialError = (error - lastError)/(currentTime - previousTime); // calculate rate of change of error
  
  lastError = error;
  previousTime = currentTime;

  if (abs(error) > 1) // prevent too much error is carry forward over course correction
  {
    errorSum = 0;
  }

  float proportional = error * Kp;  // Calculate the components of the PID
  
  float integral = errorSum * Ki;
 
  float differential = differentialError * Kd;

  double output = proportional + integral + differential;  // Calculate the result

  return output; 
}




