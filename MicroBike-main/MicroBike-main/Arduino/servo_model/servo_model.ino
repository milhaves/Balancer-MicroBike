
#include <Servo.h>
#include <Wire.h>


#define servocounts2rad 0.0089
#define servoZeroCounts 29


float servoCommand = 0; //value in radians of commanded servo position
float servoFeedback = 0; //value in radians of feedback signal
int feedbackPin = 2;
int servoPin = 5;
//create a time variable
unsigned long runTime = millis()/1000;


Servo servo;

void setup() {
  // put your setup code here, to run once:
  //have to initialize the serial port
Serial.begin(115200);
//servo.attach() turns on the servo position control.
servo.attach(servoPin);
servo.write(0);

 }


void loop() {
unsigned long runTime = millis()/1000;

if (runTime<=5)
  {
   servoCommand=0;
  
  }
  else
  {
    servoCommand=0.1;
  
  }

float feedbackValue =analogRead(feedbackPin);
//float feedbackrad = -servocounts2rad*(analogRead(feedbackPin)-servoZeroCounts);


if (runTime <= 10)
{
//float feedbackrad = analogRead(feedbackPin);
servo.write((servoCommand*180)/3.14159);
//float feedbackrad = analogRead(feedbackPin);
float feedbackrad = servocounts2rad*(analogRead(feedbackPin)-servoZeroCounts);


Serial.print(runTime);
Serial.print("\t");
Serial.print(servoCommand);
Serial.print("\t");
Serial.print(feedbackrad,4);
Serial.println();
}
}

