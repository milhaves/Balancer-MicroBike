from numpy import *
from matplotlib.pyplot import *
from scipy import signal
import control
import control.matlab as cnt

def main():
    data = loadtxt('', delimiter = ",")
    timeData = data[:,0]
    goalRollData = data[:,1]
    rollData = data[:,2]
    rollRateData = data[:,3]
    steerData = data[:,5]
    speedData = data[:,6]

    tsim = linspace(0,20,1000)
    xdesired = zeros((len(tsim),1))
    xdesired[:,0] = 0.15 #same as stepMag in PID_Control_fig8.py

    Kstep = 0.2
    wn = 23
    z = 1
    P = control.TransferFunction(Kstep*wn*wn,[1,2*z*wn,wn*wn])
    tsimVirt = linspace(0,20,1000)
    xVirt  = zeros((len(tsimVirt),1))
    xVirt,tsimVirt,something = cnt.lsim(P,xdesired,tsim)

    

if __name__ == '__main__':
    main()
