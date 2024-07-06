from controller import Robot, Motor, InertialUnit
from numpy import *

robot = Robot()

recordData = True
stepMag = 0.15
goalRoll = 0
goalRoll_tau = 0.5
goalRoll_filt = 0

driveVelocity = 1.5
Rrw = 0.027
driveOmega = driveVelocity/Rrw

if recordData:
    # start a file we can use to collect data
    f = open('webots_data.txt','w')
    f.write("# time, goalRoll, roll, rollrate, goalSteer, steer, speed \r\n")

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
<<<<<<< HEAD
kp = 1.95;
ki = 6;
kd = 0.15;
=======
# kp = 2;
# ki = 20;
# kd = 0.1;
#control gains for roll control  (PID)
kp = 2.42;
ki = 5.8;
kd = .22;
>>>>>>> 3bce3aef38eb9209a15435d6ec8036d1e1a23bfc

eRoll = 0;
eRoll_old = 0;
intE = 0;

goalRoll = 0

while robot.step(timestep) != -1:
    simtime+=timestep/1000.0

    if simtime<5:
        goalRoll = 0
    else:
        goalRoll = stepMag

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

    #filter goal roll angle to prevent crashing!
    goalRoll_filt += (timestep/1000.0)/goalRoll_tau*(goalRoll-goalRoll_filt)

# <<<<<<< HEAD
    steer.setControlPID(100,10,0)
# =======
    # steer.setControlPID(1000,0,1000)
# >>>>>>> 3bce3aef38eb9209a15435d6ec8036d1e1a23bfc
    steer.setVelocity(10)#set MAX velocity of MG90s servo
    steer.setAvailableTorque(1000)#set MAX torque of MG90S

    eRoll = goalRoll_filt - roll
    intE += (timestep/1000.0)*(eRoll)

    delta = kp*eRoll+ki*intE-kd*rollRate
    steer.setPosition(delta)

    balance.setPosition(0)

    if(recordData and simtime>=5):
        #f.write("# time, goalRoll, roll, rollrate, goalSteer, steer, speed \r\n")
        f.write(str(simtime- stepTime)+","+str(stepMag)+","+str(roll)+","+str(rollRate)+","+str(delta)+","+str(steerangle)+","+str(U)+"\r\n")
