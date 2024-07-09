"""basic_drive_controller controller."""

from controller import Robot, Motor, InertialUnit
from numpy import *

# create the Robot instance.
robot = Robot()

recordData = True
# stepTime = 2 #seconds, time at which goal roll changes
stepTime = 0
stepMag = 0.15 #radians, magnitude of change in goal roll
goalRoll_tau = 0.5 #seconds. Filter changes in goal roll angle to prevent asking for too fast of change
goalRoll = 0
goalRoll_filt = 0

wn_bal = 23 #simulated balancer PID guy
z_bal = 1.0 # simulated balancer PID guy
lean_sim = 0 #simulated lean angle
leandot_sim = 0 #simulated lean velocity
goal_lean = 0
lean = 0
oldlean = -0.01 ############################ Make sure to change this later!

lastControlTime = 0
dTcontrol = 0.0005

#set the simulation forward speed and calculate rear wheel omega
# driveVelocity= 1.5#3.95#28.95
driveVelocity = 0
Rrw = 0.027 #microbike rear wheel diameter
driveOmega = driveVelocity/Rrw

Klqr_balance = array([11.482744698711205,4.111493889816979,1.9402704228496588,0.5144317826490733]) #R=1000
# Klqr_balance = array([13.65258171555482,5.1895422344779165,2.314584057673394,0.6537620854556571]) #R=100
# Klqr_balance = array([24.360423779602062,10.477680282605482,4.162128552835325,1.3363652894025462]) #R=10
# Klqr_balance = array([63.35328842893674,29.478794880874396,10.887856437132848,3.780641295110953]) #R=1

if recordData:
    # start a file we can use to collect data
    f = open('webots_data.txt','w')
    f.write("# time, goalRoll, roll, rollrate, goalSteer, steer, speed, lean, goalLean, lean_sim \r\n")

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
# turnTime = 2*pi*R/driveVelocity+ts
T1 = Timer(5000)
print("figure 8 radius: "+str(R))
# print("figure 8 turn time: "+str(turnTime))

#now, create fsm object
fsm = fig8FSM()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Set up sensors and motors on the bike:
motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
balance = robot.getDevice('balance_servo')

motor.setPosition(float('inf'))

imu = robot.getDevice('imu')
imu.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
gyro = robot.getDevice('gyro')
gyro.enable(timestep)

steersensor = robot.getDevice('steer_angle')
steersensor.enable(timestep)
balancesensor = robot.getDevice('pendulum_position')
balancesensor.enable(timestep)


#initialize variables needed for measurement and control
simtime = 0.0
steerangle = 0



#control gains for roll control  (PI)
kp = 2.42;
# kp = 1.98
ki = 5.8;
# ki = 4.32
kd = .22;
# kd = 0.18

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
    lean = balancesensor.getValue()
    leanrate = (lean-oldlean)/(timestep/1000.0)
    oldlean = lean


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
        # print("straight: "+str(T0.elapsed))
    elif fsm.TURNRIGHT:
        goalRoll = 0
        # print("turn right: "+str(T1.elapsed))
    elif fsm.TURNLEFT:
        goalRoll =stepMag
        # print("turn left: "+str(T1.elapsed))

    goalRoll = 0

    if((simtime-lastControlTime)>dTcontrol):

        #filter goal roll angle to prevent crashing!
        goalRoll_filt+= (timestep/1000.0)/goalRoll_tau*(goalRoll-goalRoll_filt)

        #step in balance goal lean
        # if(simtime>stepTime):
        #     goal_lean = 0.5

        # goal_lean = -(Klqr_balance[0]*roll +Klqr_balance[1]*lean + Klqr_balance[2]*rollRate + Klqr_balance[3]*leanrate)
        goal_lean = Klqr_balance[0]*roll - Klqr_balance[1]*lean - Klqr_balance[2]*rollRate - Klqr_balance[3]*leanrate

        #now compute true lean based on goal lean and servo dynamics
        lean_sim+= timestep/1000.0*leandot_sim
        leandot_sim+=  timestep/1000.0*(wn_bal*wn_bal*(goal_lean-lean_sim) - z_bal*wn_bal*leandot_sim)


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
        # steer.setPosition(delta)
        steer.setPosition(0)
        balance.setPosition(lean_sim)
        # balance.setPosition(0)

        print("Goal Roll: "+str(goalRoll))
        print("Roll: "+str(roll))
        print("Goal Lean: "+str(goal_lean))
        print("Lean: "+str(lean))
        print("Lean_sim: "+str(lean_sim))
        print("##########################")
        lastControlTime = simtime

    if(recordData and simtime>=stepTime):
        #f.write("# time, goalRoll, roll, rollrate, goalSteer, steer, speed \r\n")
        f.write(str(simtime- stepTime)+","+str(stepMag)+","+str(roll)+","+str(rollRate)+","+str(delta)+","+str(steerangle)+","+str(U)+","+str(lean)+","+str(goal_lean)+","+str(lean_sim)+"\r\n")
