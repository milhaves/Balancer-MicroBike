
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

Servo myServo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int pos = 0;    // variable to store the servo position

float rollover = 0.0;//variable to correct for 180 to 0 IMU skips
float rollFinal = 0;
float rollLast = 0;

void setup() {
  Serial.begin(115200);
  if (!bno.begin())
  {
    Serial.print("No BNO055 detected");
//    while (1);
  }
  
  myServo.attach(5  );  // attaches the servo on pin 9 to the servo object
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

  Serial.begin(115200);
  delay(1000);

 

}

void loop() {

  //get IMU data
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  float roll = orientationData.orientation.y;
  //check for rollover and correct
  if((roll-rollLast)>100){
    rollover-=1.0;
  }
  else if((roll - rollLast)<-100){
    rollover+=1.0;
  }

  //store old value of raw roll
  rollLast = roll;
  //compute final roll angle inclusive of rollover correction
  rollFinal = roll + rollover*180.0;

  //drive motor: full speed
  digitalWrite(9,HIGH);
  digitalWrite(10,HIGH);

  //set steering to K*rollFinal (p-control on roll angle)
  float K = -1.0;
  float steer = K*roll;

  //set steering motor to computed angle
  myServo.write(90+steer);

  //print to serial port for debugging
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(rollFinal);
  Serial.print("\t");
  Serial.println(rollover);
  
  
}
