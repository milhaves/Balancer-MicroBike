/* 
Basic interface code to command and view feedback from a modified servo
(modified to break out analog feedback signal)

copyright Alexander Brown 2024 brownaa@lafayette.edu
*/

//include the Servo library so we can drive a servo
#include <Servo.h>
#include <Wire.h>

//MIN and MAX PWM pulse sizes for servos.
//These can be found in servo documentation (datasheet)
#define MAX 2200
#define MIN 800

//define constants for conversions
#define pi  3.14159
#define deg2rad 180/pi
#define servocounts2rad 0.003
#define servoZeroCounts 327

//define variables that will be used
float enableServo = 1.0;  //1 or 0, 1 enables the servo.
float servoCommand = 1; //value in radians of commanded servo position
float servoFeedback = 0; //value in radians of feedback signal
int feedbackPin = 2;
int servoPin = 5;
//create a time variable
unsigned long runTime;

//define a servo object and specify which arduino digital pin it is connected to
Servo servo;
//the microsecond pulse length corresponding to 0 degrees on your servo (adjust)
static int zero_uS=1600;
int servo_cmd_us = 1470;

//maximum servo positions, 0 is horizontal position
float servo_min=radians(-80);
float servo_max=radians(80);

//servo_mult - multiplier used for conversion radians->servo pulse in us
//you may need to adjust this!
float servo_mult=1000/(pi/4);

void setup(){
  
//have to initialize the serial port
Serial.begin(115200);
//servo.attach() turns on the servo position control.
servo.attach(servoPin);

}


//main control loop, obtain requested action from serial connection, then execute it
void loop()
{ 
  while(Serial.available()>0){
    //a command will start with a ! character
    if(Serial.read()=='!'){
      //first number will be 1.0 or 0.0. If 0, disable the servo.
      enableServo = Serial.parseFloat();
      //second number will be the command for the servo in radians.
      servoCommand = Serial.parseFloat();
      
      //If given command is greater than the max value, acts at max value.
      if (servoCommand>servo_max){
       servoCommand = servo_max;
      }
      else if(servoCommand<servo_min){
        servoCommand = servo_min;
      }
      float feedbackrad = servocounts2rad*(analogRead(feedbackPin)-servoZeroCounts);
      //now send feedback for this command.
      //send the time since powered on in milliseconds
      Serial.print(millis());
      Serial.print(",");
      Serial.print(feedbackrad);
      Serial.println();
    }
    }

    //now we can take our latest servo command, convert to uS, write to servo if enabled.
    if(1){    
      //write the number of microseconds to the servo.
      servo_cmd_us = constrain(zero_uS + servoCommand*servo_mult, MIN,MAX);
      servo.writeMicroseconds(servo_cmd_us);
    }
    else{
      servo.detach();
    }
    
    //use the delay() function to artificially limit how fast the loop runs.
    delay(1);

}
