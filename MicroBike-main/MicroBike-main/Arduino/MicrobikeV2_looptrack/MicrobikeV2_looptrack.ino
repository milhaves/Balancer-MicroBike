
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Servo.h>

Servo myServo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
int pos = 0;    // variable to store the servo position

//FSM variables
float Unom = 0.75;//meters per sec, fwd speed
float Rturn =0.75;//meters, desired turn radius
float angle_turn = 20*PI;//180 degree turns
float straightLen = 2;//desired straight length
float ts = 0.5;//guess for settling time
float rollDesired_rad = 0.15;//atan(Unom*Unom/(9.81*Rturn));
float Tturn_pre = ts+angle_turn*Rturn/Unom;
float Tstraight_pre = 3.0;//straightLen/Unom;
float Tturn_start = 0;
float Tstraight_start = 0;
float Tturn_elapsed = 0;
float Tstraight_elapsed = 0;
float desFilt_tau = 0.5;//seconds, time constant for filter on goal roal
float goalRoll_filt = 0;

bool WAIT=true;
bool STRAIGHT=false;
bool TURN=false;

float rollover = 0.0;//variable to correct for 180 to 0 IMU skips
float rollFinal = 0;
float rollLast = 0;

//controller design
float steerCenterAngle = 100;
float Kp = 2;// kp=2,ki=25,kd=5 worked ok
float Ki =25;//25;//25;
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
  oldCounts = eCountsSaved;
  //if we push bike by hand, U will be large. Then start motor.
  if(!motorGo&&U>0.5){
    motorGo=true;
  }

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

  //timers for FSM
  if(!STRAIGHT){
    Tstraight_start = t;
    Tstraight_elapsed=0;
  }
  else{
    Tstraight_elapsed = t-Tstraight_start;
  }
  if(!TURN){
    Tturn_start = t;
    Tturn_elapsed = 0;
  }
  else{
    Tturn_elapsed = t-Tturn_start;
  }

  //now FSM: set truth value of transitions
  bool L1 = WAIT&&!motorGo;//transition from wait to wait (latch)
  bool T1 = WAIT&&motorGo;//transition from wait to straight
  bool L2 = STRAIGHT&&Tstraight_elapsed<Tstraight_pre;//latch on straight
  bool T2 = STRAIGHT&&Tstraight_elapsed>=Tstraight_pre;//transition from straight to turn
  bool L3 = TURN&&Tturn_elapsed<Tturn_pre;//latch on turn
  bool T3 = TURN&&Tturn_elapsed>=Tturn_pre;//transition from turn to straight

  //set truth value of states
  WAIT = L1;
  STRAIGHT = T1||T3||L2;
  TURN = L3||T2;
  

  //now outputs (controller etc.)
  
  
  //if straight, goal roll is 0. If turning, it's not.
  if(!TURN){
    rollDesired = 0;
  }
  else{
    rollDesired = rollDesired_rad*180/PI;
  }
  //compute the filtered goal roll
  goalRoll_filt+= dt/desFilt_tau*(rollDesired -  goalRoll_filt);


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
  float e = goalRoll_filt - rollFinal;
  //compute error integral
  intE+=dt*e; 
  //constrain error integral to make sure it doesn't get huge
  intE = constrain(intE,-10.0/Ki,10.0/Ki);//with limit integral corrections to +/- 1 degree
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

  if(1){
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
    Serial.print("\t");
    Serial.print(goalRoll_filt);
    Serial.print("\t");
    Serial.print(WAIT);
    Serial.print("\t");
    Serial.print(STRAIGHT);
    Serial.print("\t");
    Serial.print(TURN);
    Serial.print("\t");
    Serial.print(Tstraight_elapsed);
    Serial.print("\t");
    Serial.print(Tturn_elapsed);
    Serial.print("\t");
    Serial.print(Tstraight_start);
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
  
  
}


void isr(){
  eCounts++;
}
