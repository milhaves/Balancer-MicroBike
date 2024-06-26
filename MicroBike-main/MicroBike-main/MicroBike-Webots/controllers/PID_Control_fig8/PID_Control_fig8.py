"""basic_drive_controller controller."""

from controller import Robot, Motor, InertialUnit
from numpy import *

# create the Robot instance.
robot = Robot()

recordData = True
stepTime = 2 #seconds, time at which goal roll changes
stepMag = 0.15 #radians, magnitude of change in goal roll
goalRoll_tau = 0.5 #seconds. Filter changes in goal roll angle to prevent asking for too fast of change
goalRoll = 0
goalRoll_filt = 0

#set the simulation forward speed and calculate rear wheel omega
driveVelocity= 1.5#3.95#28.95
Rrw = 0.027 #microbike rear wheel diameter
driveOmega = driveVelocity/Rrw

if recordData:
    # start a file we can use to collect data
    f = open('webots_data.txt','w')
    f.write("# time, goalRoll, roll, rollrate, goalSteer, steer, speed \r\n")

#timer class for switching figure 8 directions
class Timer:
    def __init__(self,preset):
        #current "state" of the timer
        self.state = False
        #current elapsed time
        self.elapsed = 0
        #timer will go true if it has counted for more than 1 second
        self.preset = preset
    def update(self,ENABLE,dt):
        #dt is the timestep by which we should count up. make sure its units match your preset!
        #don't set the preset in seconds and increment the elapsed time in milliseconds, for example!
        #ENABLE is a boolean. When it is true, we run up the timer. When it is not, the time resets and we stop counting.
        if(ENABLE):
            #increment time by dt
            self.elapsed+=dt
            self.state=self.elapsed>=self.preset
        else:
            self.elapsed=0
            self.state=False


#FSM for figure 8s.
class fig8FSM:
    def __init__(self):
        #initialize FSM to start in wait
        self.STRAIGHT = True
        self.TURNLEFT = False
        self.TURNRIGHT = False


    def update(self,T0,T1):
        #latch on straight, waits for T0 to expire
        t1 = self.STRAIGHT and not T0
        #transition from straight to turnleft.
        t2 = self.STRAIGHT and T0
        #transition from turnleft to turnright
        t3 = self.TURNLEFT and T1
        #latch on turnleft
        t4 = self.TURNLEFT and not T1
        #latch on turn right
        t5 = self.TURNRIGHT and not T1
        #transition from turnright to turnleft
        t6 = self.TURNRIGHT and T1


        #block 3: set States
        self.STRAIGHT = t1
        self.TURNLEFT = t2 or t4 or t6
        self.TURNRIGHT = t3 or t5



#now create a timer 1 object (straight timer) and timer 2 object (turn timer)
#timer presets are in milliseconds
T0 = Timer(1000*stepTime) #give the bike time to settle in straight
#compute turn radius from step roll value, g, and fwd speed
g = 9.81 #gravity
R = driveVelocity**2/(g*tan(stepMag))
ts = 0.5 #settling time of roll dynamics (slowest eigenvalue)
#now that we know turn radius, find out how long to complete a turn:
turnTime = 2*pi*R/driveVelocity+ts
T1 = Timer(5000)
print("figure 8 radius: "+str(R))
print("figure 8 turn time: "+str(turnTime))

#now, create fsm object
fsm = fig8FSM()

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


#initialize variables needed for measurement and control
simtime = 0.0
steerangle = 0



#control gains for roll control  (PI)
kp = 2.42;
ki = 5.8;
kd = .22;

eRoll = 0;
eRoll_old = 0;
intE = 0;

goalRoll = 0

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

    #update timers
    T0.update(fsm.STRAIGHT,timestep)
    T1.update(((fsm.TURNLEFT or fsm.TURNRIGHT)and not T1.state),timestep)
    #now update FSM
    fsm.update(T0.state,T1.state)

   #change goal roll based on fsm state
    if fsm.STRAIGHT:
        goalRoll = 0
        print("straight: "+str(T0.elapsed))
    elif fsm.TURNRIGHT:
        goalRoll = stepMag
        print("turn right: "+str(T1.elapsed))
    elif fsm.TURNLEFT:
        goalRoll =-stepMag
        print("turn left: "+str(T1.elapsed))

    #filter goal roll angle to prevent crashing!
    goalRoll_filt+= (timestep/1000.0)/goalRoll_tau*(goalRoll-goalRoll_filt)


    #print(roll)
    #set the PID gains on the steer servo
    #TODO: compute these for the REAL servo given step response of steer by itself.
    steer.setControlPID(100,10,0)
    steer.setVelocity(10)#setw MAX velocity of MG90s servo
    steer.setAvailableTorque(.215)#set MAX torque of MG90S

    eRoll = goalRoll_filt -roll
    intE += (timestep/1000.0)*(eRoll)

    #compute steer angle command based on control law
    #note that steering has a servo, so its ACTUAL steer angle is different than this command.
    delta = kp*eRoll+ki*intE-kd*rollRate
    steer.setPosition(delta)

    if(recordData and simtime>=stepTime):
        #f.write("# time, goalRoll, roll, rollrate, goalSteer, steer, speed \r\n")
        f.write(str(simtime- stepTime)+","+str(stepMag)+","+str(roll)+","+str(rollRate)+","+str(delta)+","+str(steerangle)+","+str(U)+"\r\n")
