from numpy import *
from matplotlib.pyplot import *
from scipy import signal
import control
import control.matlab as cnt

def main():
    data = loadtxt('webots_data.txt', delimiter = ",")
    timeData = data[:,0]
    goalRollData = data[:,1]
    rollData = data[:,2]
    rollRateData = data[:,3]
    steerData = data[:,5]
    speedData = data[:,6]

    tsim = linspace(0,3,1000)
    xdesired = zeros((len(tsim),1))
    xdesired[:,0] = 0.15 #same as stepMag in PID_Control_fig8.py

    goalRoll = 0.15

    # # Kstep = 0.2
    # # wn = 15
    # # z = 2.5
    # wn = 23
    # z = 1
    # P = control.TransferFunction(wn*wn,[1,2*z*wn,wn*wn])
    # tsimVirt = linspace(0,3,1000)
    # xVirt  = zeros((len(tsimVirt),1))
    # xVirt,tsimVirt,something = cnt.lsim(P,xdesired,tsim)

    g = 9.81
    m1 = 0.2
    m2 = 0.05
    l1 = 0.1
    lc1 = 0.04
    l2 = 0.05
    lc2 = l2

    I1 = (1/3)*m1*lc1*lc1
    I2 = (1/3)*m2*(l1+lc2)**2

    s1 = -3.393 #taken fron PIDcontrol_design.m
    s2 = -3.393

    J = I1+I2

    bvirtual = -1*J*(s1+s2)
    Kvirtual = J*s1*s2
    mtot = m1+m2

    square = (rollData[-1]/goalRoll)*s1*s2 #steady state gain

    print('######### J #########')
    print(J)
    print('######### bvirtual #########')
    print(bvirtual)
    print('######### Kvirtual #########')
    print(Kvirtual)

    P = control.TransferFunction(square,[1,bvirtual/J,Kvirtual/J])
    tsimVirt = linspace(0,3,1000)
    xVirt  = zeros((len(tsimVirt),1))
    xVirt,tsimVirt,something = cnt.lsim(P,xdesired,tsim)

    figure()
    subplot(2,1,1)
    title("Steer Control w/ Locked Balancer CLSR: Desired Roll = "+"{:.2f}".format(goalRoll)+" radians")
    plot(tsim,goalRoll*ones((len(tsim),1)),'k--',timeData,rollData,'k',tsimVirt,xVirt,'r')
    xlabel('Time (s)')
    ylabel('Roll Angle (rad)')
    legend(['Desired Roll Angle', 'Webots Step Response', 'Modeled Step Response'])
    subplot(2,1,2)
    plot(timeData,steerData,'k')
    ylabel('Steer Angle (rad)')
    show()

if __name__ == '__main__':
    main()
