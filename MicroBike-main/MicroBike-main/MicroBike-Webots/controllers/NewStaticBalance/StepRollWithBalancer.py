from controller import Robot, Motor, InertialUnit
from numpy import *
from matplotlib.pyplot import *
import control
import control.matlab as cnt

sys.path.insert(0,'../utilities')
from Rollover import Rollover
from actuator_simulator import actuator_simulator

# set up steering actuator
steer_zeta = 1.0
steer_wn = 23.0
steer_actuator = actuator_simulator(steer_zeta,steer_wn)

balance_zeta = 1.0
balance_wn = 23.0
balance_actuator = actuator_simulator(balance_zeta,balance_wn)

# create the Robot instance
robot = Robot()
yawCorr = Rollover()

# control params
lastControlTime = 0
dTcontrol = 0.0005
oldsteer = 0

# sim values
roll_stepTime = 2
roll_stepVal = 0.15
roll_desired_tau = 0.5
roll_desired_filt = 0

goal_lean = 0
lean = 0
oldlean = 0

driveVelocity = 1.5
Rrw = 0.027
driveOmega = driveVelocity/Rrw

K = array([12.187552435885927,4.533736970673231,2.1011730141790705,0.6481864326278665])

recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('./webots_data.txt','w')
    f.write("# time, roll_desired, roll, steer_desired, steer, lean_desired, lean \r\n")

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

kp = 2.1
ki = 3.15
kd = 0.35
Ksum = 1.0

eRoll = 0
intE = 0

T = 0
simtime = 0.0
firstLoop = True

# main loop
while(robot.step(timestep) != -1) and simtime<(roll_stepTime*2):
    simtime+=timestep/1000.0
    if(firstLoop):
        oldlean = balancesensor.getValue()
        firstLoop = False
    if(simtime>=roll_stepTime):
        roll_desired = roll_stepVal
    else:
        roll_desired = 0

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

    lean = balancesensor.getValue()
    leanrate = (lean-oldlean)/(timestep/1000.0)
    oldlean = lean

    # set motor to correct velocity
    motor.setVelocity(driveOmega)

    roll_desired_filt += (timestep/1000.0)/roll_desired_tau*(roll_desired-roll_desired_filt)

    intE += (timestep/1000.0)*(eRoll)
    eRoll = roll_desired_filt - roll

    delta = Ksum*(kp*eRoll+ki*intE-kd*rollRate)

    # update the command steer angle using the acutator model
    steer_actuator.update(delta,timestep/1000.0)

    # goal_lean = -(K[0]*roll + K[1]*lean + K[2]*rollRate + K[3]*leanrate)
    goal_lean = K[0]*roll - K[1]*lean - K[2]*rollRate - K[3]*leanrate

    balance_actuator.update(goal_lean,timestep/1000.0)

    if((simtime-lastControlTime)>dTcontrol):
        steer.setControlPID(1000,0,0)
        steer.setVelocity(1000)
        steer.setAvailableTorque(100)

        balance.setControlPID(1000,0,0)
        balance.setVelocity(1000)
        balance.setAvailableTorque(100)

        steer.setPosition(steer_actuator.delta)
        print("Steer: "+str(steer_actuator.delta))
        balance.setPosition(balance_actuator.delta)
        print("Balancer: "+str(balance_actuator.delta))
        print("#############################")

        lastControlTime = simtime
    if(recordData):
        f.write(str(simtime)+","+str(roll_desired)+","+str(roll)+","+str(delta)+","+str(steerangle)+str(goal_lean)+","+str(lean)+"\r\n")
