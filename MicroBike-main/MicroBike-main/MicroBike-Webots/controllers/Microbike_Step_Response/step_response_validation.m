%# time, goalRoll, roll, rollrate, goalSteer, steer, speed 
clear all
close all
clc

%load webots data
d=load('webots_data_ml.txt');
t = d(:,1);
roll = d(:,3);

%set up TF for webots controller
h = 0.04;
a = 0.07;
b = .152;
c = 0.008;
lam = 1;
U = mean(d(:,7));
g = 9.81;
kp = 4;
stepMag = .1;

s = tf('s');
%set up open loop TF
P = minreal(sin(lam)/b* (a/h*U*s + U^2/h - g*a*c/(h*U))/(s^2-g/h))
%compute closed loop TF at our kp:
Gcl = minreal(kp*P/(1+kp*P));

%compute step response of closed loop system
[ysim,tsim] = step(Gcl,max(t));
%scale step response by goal roll magnitude
ysim = ysim*stepMag;

%plot results
figure
plot(t,roll,'k',tsim,ysim,'r')
legend('webots','linear model')
xlabel('Time (s)')
ylabel('Roll (rad)')
xlim([0 1])
