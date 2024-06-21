
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

//controller design
float steerCenterAngle = 100;
float Kp = 2;// kp=2,ki=25,kd=5 worked ok
float Ki =25;//25;
float Kd = 10;
float intE = 0;
float rollDesired = 0;//this should be read from serial eventually
float t=0;
float told=0;
float dt=.01;

//if we push wheel, start going.
bool motorGo = false;

volatile long eCounts = 0;
long eCountsSaved = 0;
long oldCounts = 0;

void setup() {
pinMode(7,INPUT);
//pinMode(11,INPUT);
  attachInterrupt(digitalPinToInterrupt(7),isr,CHANGE);
  Serial.begin(115200);
  Serial1.begin(9600);
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

 told=micros()/1.0e6;

}

void loop() {
  //turn off interrupts so we can save counts from encoder
  noInterrupts();
  eCountsSaved = eCounts;
  interrupts();
  
  t=micros()/1.0e6;
  dt-t-told;
  told=t;

  //compute speed
  float U = .0254*2*PI/300*(eCountsSaved-oldCounts)/dt;
  //if we push bike by hand, U will be large. Then start motor.
  if(!motorGo&&U>0.5){
    motorGo=true;
  }
  
  oldCounts = eCountsSaved;
  
  //get IMU data
  sensors_event_t orientationData, gravityData, angVelocityData;
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  float roll = orientationData.orientation.z;
  float rollRate = angVelocityData.gyro.x;
  //check for rollover and correct
  if((gravityData.acceleration.z)<0){
    rollFinal = 89-roll;
  }
  else{
    rollFinal = 89-roll;
  }


  //drive motor: full speed
  //only if the motorgo command is true.
  if(motorGo){
    digitalWrite(9,HIGH);
    digitalWrite(10,HIGH);
  }
   else{
    digitalWrite(9,LOW);
    digitalWrite(10,LOW);
   }
  
  //compute error
  if(motorGo){
  float e = rollDesired - rollFinal;
  //compute error integral
  intE+=dt*e; 
  //constrain error integral to make sure it doesn't get huge
  intE = constrain(intE,-25.0/Ki,25.0/Ki);//with limit integral corrections to +/- 1 degree
  //compute steer angle based on control law
  float steer = steerCenterAngle - (Kp*e-Kd*rollRate+Ki*intE); //note 90 minus. This depends on direction servo actually turns with positive angle.
  //constrain steer to avoid hitting th steering stops
  steer = constrain(steer,40,140);//make sure angle doesn't exceed +/- 30 deg from center

  //set steering motor to computed angle
  myServo.write(steer);
  }
  else{
    myServo.write(steerCenterAngle);
  }
  Serial1.println(millis());
  Serial1.print(",");
  Serial1.print(rollFinal);
  Serial1.println();
//  Serial1.print("\r\n");
  //print for debugging
  Serial.print(U);
  Serial.print("\t");
  Serial.print(rollFinal);
  Serial.print("\t");
  Serial.print(rollRate);
  Serial.print("\t");
  Serial.print(dt);
  Serial.print("\t");
  Serial.print(intE);
  Serial.println();
  //print to serial port for debugging
//  Serial.print(rollFinal);
//  Serial.print("\t");
//  Serial.print(gravityData.acceleration.x);
//  Serial.print("\t");
//  Serial.print(gravityData.acceleration.y);
//  Serial.print("\t");
//  Serial.print(gravityData.acceleration.z);
//  Serial.print("\t");
//  Serial.println();
  
  
}


void isr(){
  eCounts++;
}
