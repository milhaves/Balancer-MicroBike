from RazorModel import getRollDLQRRazor
from numpy import *

vels = arange(1,7,0.5)
for k in range(0,len(vels)):
    sys,Klqr = getRollDLQRRazor(vels[k],0.005,0.05)
    allGains = ravel(Klqr)
    if(k==0):
        gainmtx = hstack((vels[k],allGains))
    else:
        gainmtx = vstack((gainmtx,hstack((vels[k],allGains))))

savetxt('razorGains_dlqr.txt',gainmtx,comments='# generated by generateLQRvals.py referencing RazorModel.py')