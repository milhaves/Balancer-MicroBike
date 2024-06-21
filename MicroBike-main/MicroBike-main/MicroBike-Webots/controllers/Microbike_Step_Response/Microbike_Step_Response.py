"""basic_drive_controller controller."""

from controller import Robot, Motor, InertialUnit
from numpy import *

# create the Robot instance.
robot = Robot()

recordData = True
stepTime = 2 #seconds, time at which goal roll changes
stepMag = 0.1 #radians, magnitude of change in goal roll

if recordData:
    # start a file we can use to collect data
    f = open('webots_data.txt','w')
    f.write("# time, goalRoll, roll, rollrate, goalSteer, steer, speed \r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Set up sensors and motors on the bike:
motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
motor.setPosition(float('inf'))

imu = robot.getDevice('imu')
imu.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
gyro = robot.getDevice('gyro')
gyro.enable(timestep)

steersensor = robot.getDevice('steer_angle')
steersensor.enable(timestep)


#initialize variables needed for meeasurement and control
simtime = 0.0
steerangle = 0

#set the simulation forward speed and calculate rear wheel omega
driveVelocity= 1#3.95#28.95
Rrw = 0.027 #microbike rear wheel diameter
driveOmega = driveVelocity/Rrw

#control gains for roll control  (PD)
k=4;
kd = 0;

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    simtime+=timestep/1000.0
    
    # Read the sensors:
    #get current fwd speed
    U = gps.getSpeed()

    #read IMU values
    rpy = imu.getRollPitchYaw()
    gyros = gyro.getValues()
    #pull out the roll and roll rate from the IMU values
    roll = rpy[0]
    rollRate = gyros[0]

    #read the actual steer angle from steering servo feedback
    steerangle = -steersensor.getValue()
    oldsteer = steerangle


    # set the motor to the correct velocity:
    # TODO: use closed-loop PI control and model the actual motor.
    motor.setVelocity(driveOmega)
    
   #change goal roll if enough time has passed 
    if(simtime>stepTime):
       goalRoll = stepMag
    else:
       goalRoll = 0
    
    
    #set the PID gains on the steer servo
    #TODO: compute these for the REAL servo given step response of steer by itself.
    steer.setControlPID(100,10,0)
    
    #compute steer angle command based on control law
    #note that steering has a servo, so its ACTUAL steer angle is different than this command.
    delta = k*(goalRoll - roll)+kd*rollRate
    steer.setPosition(delta)
        
    if(recordData and simtime>=stepTime):
        #f.write("# time, goalRoll, roll, rollrate, goalSteer, steer, speed \r\n")
        f.write(str(simtime- stepTime)+","+str(stepMag)+","+str(roll)+","+str(rollRate)+","+str(delta)+","+str(steerangle)+","+str(U)+"\r\n")
        