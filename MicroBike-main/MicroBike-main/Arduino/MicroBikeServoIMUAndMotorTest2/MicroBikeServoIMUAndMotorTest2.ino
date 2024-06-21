
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
  
  myServo.attach(5);  // attaches the servo on pin 9 to the servo object
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);

  Serial.begin(115200);
  delay(1000);

 

}

void loop() {

  //get IMU data
  sensors_event_t orientationData, gravityData, angVelocityData;
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  float roll = orientationData.orientation.y;
  float rollRate = angVelocityData.gyro.y;
  //check for rollover and correct
  if((gravityData.acceleration.z)<0){
    rollFinal = 89-roll;
  }
  else{
    rollFinal = roll-89;
  }

//  //store old value of raw roll
//  rollLast = roll;
//  //compute final roll angle inclusive of rollover correction
//  rollFinal = roll + rollover*180.0;

  //drive motor: full speed
  digitalWrite(9,HIGH);
  digitalWrite(10,HIGH);

  //set steering to K*rollFinal (p-control on roll angle)
  float K = 4.0;
  float Kd = -5;
  float steer = K*rollFinal+Kd*rollRate;

  //set steering motor to computed angle
  myServo.write(90+steer);

  //print to serial port for debugging
  Serial.print(rollFinal);
  Serial.print("\t");
  Serial.print(gravityData.acceleration.x);
  Serial.print("\t");
  Serial.print(gravityData.acceleration.y);
  Serial.print("\t");
  Serial.print(gravityData.acceleration.z);
  Serial.print("\t");
  Serial.println();
  
  
}
