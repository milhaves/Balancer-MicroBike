"""Motorcycle path following controller."""

from controller import Robot, Motor, InertialUnit
from numpy import *
from matplotlib.pyplot import *
import control
import control.matlab as cnt
#add paths for custom libraries
sys.path.insert(0, '../utilities')
from realtime_plotter import RealTimePlot
from MC_model import *
from maptools import *
from pointmassracer import *
from Rollover import Rollover
from actuator_simulator import actuator_simulator

showPlots = True

###### set up "slow" steering actuator (doing this with PID on steer motor no workie)
steer_zeta = 1 #actuator damping ratio
steer_wn = 23 #actuator natural frequency
steer_actuator = actuator_simulator(steer_zeta,steer_wn)
# plot = RealTimePlot(x_label='time',y_label='rollrate',marker='k')

# create the Robot instance.
robot = Robot()
yawCorr = Rollover()

param_names = [   'a ','b ','c','hrf','mrf','xff','zff','mff','Rfw','mfw','Rrw','mrw','Jyyf','Jyyr','lam']

#venom params:
MC_params = array([.451,.99,.0526,.506,22.717,.9246,.515,5.105,.2413,4.278,.2413,7.1,.125,.2,1.345])


# CONTROL PARAMETERS
lastControlTime = 0
dTcontrol = 0.005
oldsteer = 0

#simulation VALUES
steer_stepTime = 2.0
steer_stepVal = 0.1

def clamp(val,min,max):
    assert min<max
    if(val<min):
        val=min
    elif val>max:
        val=max
    return val


def setDriveMotorTorque(self,motor,command,omega):
    #Assume that the torque to the wheel is 650 N-m max
    #for now, just allow max torque, and assume brake is same
    motorTorque = clamp(command,-650,650)
    #set motor force
    motor.setTorque(motorTorque)

def find_nearest_index(array, value):
    array = asarray(array)
    idx = (abs(array - value)).argmin()
    return idx

def getCurrentGains(speed):
    #find the speed in the gain array closest to ours
    idx = find_nearest_index(lqrspeeds,speed)
    #find the gain set at this index
    Klqr = gainmtx[idx,1:]
    return Klqr,lqrspeeds[idx]


recordData = True

if recordData:
    # start a file we can use to collect data
    f = open('./webots_data.txt','w')
    f.write("# time, steer_desired, steer \r\n")

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
motor = robot.getDevice('drive_motor')
steer = robot.getDevice('steering_motor')
motor.setPosition(float('inf'))

motor.setPosition(float('inf'))

imu = robot.getDevice('imu')
imu.enable(timestep)
gps = robot.getDevice('gps')
gps.enable(timestep)
gyro = robot.getDevice('gyro')
gyro.enable(timestep)
steersensor = robot.getDevice('steer_angle')
steersensor.enable(timestep)


####### INITIALIZE VALUES ########
T = 0
simtime = 0.0
firstLoop = True

# Main loop:
while (robot.step(timestep) != -1) and simtime<(steer_stepTime*2):
    simtime+=timestep/1000.0
    if(firstLoop):
        firstLoop=False
    if(simtime>=steer_stepTime):
        steer_desired = 0
    else:
        steer_desired = steer_stepVal

    #update the command steer angle using the actuator model
    steer_actuator.update(steer_desired,timestep/1000.0)

    U = gps.getSpeed()

    # Read the sensors:
    #now get steer values and calculate steer rate.
    # WEBOTS is in ISO (z up) but Papa is in SAE (z down) so need to flip dir.
    steerangle = steersensor.getValue()
    steerRate = (steerangle-oldsteer)/(timestep/1000.0)
    oldsteer = steerangle
    #read IMU value
    rpy = imu.getRollPitchYaw()
    gyros = gyro.getValues()
    #rollInt += (timestep/1000.0)*rpy[0]
    yaw = rpy[2]
    yaw = yawCorr.update(yaw)
    yawRate = gyros[2]#(yaw-oldYaw)/(timestep/1000.0)
    # print("yaw/old: "+str(yaw)+","+str(oldYaw))
    oldYaw = yaw
    roll = rpy[0]
    rollRate = gyros[0]#(roll-oldRoll)/(timestep/1000.0)

    motor.setVelocity(0)
    # print("speed = "+str(U))

    if((simtime-lastControlTime)>dTcontrol):
        steer.setControlPID(100,0,0)
        steer.setPosition(steer_actuator.delta)
        print(steer_actuator.delta)
        lastControlTime = simtime
    if(recordData):
        f.write(str(simtime)+","+str(steer_desired)+","+str(steerangle)+"\r\n")

#load validation data
# data = loadtxt('data/clsteer_step_3.txt',delimiter=',')
# tdata = (data[:,0]-data[0,0])/1000.0
# steerdata = data[:,1]


wbdata = loadtxt('webots_data.txt',delimiter=',')
twb = wbdata[:,0]
deswb = wbdata[:,1]
steerwb = wbdata[:,2]
twb = twb-steer_stepTime
#create a vector for a linear simulation
steersim = zeros(size(steerwb))
#create an object for a linear simulation
newsteer_act = actuator_simulator(steer_zeta,steer_wn)
#run linear simulation
for k in range(1,len(steersim)):
    newsteer_act.update(deswb[k-1],timestep/1000.0)
    steersim[k]=newsteer_act.delta
#
figure
# plot(twb,deswb,'r',twb,steerwb,'b',twb,steersim,'g',tdata,steerdata,'k')
plot(twb,deswb,'r',twb,steerwb,'b',twb,steersim,'g')
# legend(['desired','webots','linear sim','data'])
legend(['Desired','Webots','Linear Sim'])
xlabel('Time (s)')
ylabel('Steering Angle (rad)')
xlim([0, max(twb)])

show()
