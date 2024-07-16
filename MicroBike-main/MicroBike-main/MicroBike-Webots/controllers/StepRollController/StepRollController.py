from controller import Robot, Motor, InertialUnit
from numpy import *
from matplotlib.pyplot import *
import control
import control.matlab as cnt

sys.path.insert(0,'../utilities')
from Rollover import Rollover
from actuator_simulator import actuator_simulator

# set up steering actuator
steer_zeta = 1
steer_wn = 23
steer_actuator = actuator_simulator(steer_zeta,steer_wn)

# create the Robot instance
robot = Robot()
yawCorr = Rollover()

# control params
lastControlTime = 0
dTcontrol = 0.005
oldsteer = 0

# sim values
roll_stepTime = 2
roll_stepVal = 0.15
roll_desired_tau = 0.5
roll_desired_filt = 0

driveVelocity = 1.5
Rrw = 0.027
driveOmega = driveVelocity/Rrw

recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('./webots_data.txt','w')
    f.write("# time, roll_desired, roll, steer_desired, steer \r\n")

# get timestep of current world
timestep = int(robot.getBasicTimeStep())

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

kp = 3.2065
ki = 6.996
kd = 0.2195

eRoll = 0
intE = 0

T = 0
simtime = 0.0
firstLoop = True

# main loop
while(robot.step(timestep) != -1) and simtime<(roll_stepTime*2):
    simtime+=timestep/1000.0
    if(firstLoop):
        firsLoop = False
    if(simtime>=roll_stepTime):
        roll_desired = 0
    else:
        roll_desired = roll_stepVal

    # get forward speed
    U = gps.getSpeed()

    # read IMU vals
    rpy = imu.getRollPitchYaw()
    gyros = gyro.getValues()

    # pull out roll and roll rate
    roll = rpy[0]
    rollRate = gyros[0]

    # read actual steer angle
    steerangle = -steersensor.getValue()
    oldsteer = steerangle

    # set motor to correct velocity
    motor.setVelocity(driveOmega)

    roll_desired_filt += (timestep/1000.0)/roll_desired_tau*(roll_desired-roll_desired_filt)

    eRoll = roll_desired_filt - roll
    intE += (timestep/1000.0)*(eRoll)

    delta = kp*eRoll+ki*intE-kd*rollRate

    # update the command steer angle using the acutator model
    steer_actuator.update(delta,timestep/1000.0)

    if((simtime-lastControlTime)>dTcontrol):
        steer.setControlPID(100,0,0)
        steer.setPosition(steer_actuator.delta)
        print(steer_actuator.delta)
        balance.setPosition(0)
        lastControlTime = simtime
    if(recordData):
        f.write(str(simtime)+","+str(roll_desired)+","+str(roll)+","+str(delta)+","+str(steerangle)+"\r\n")
