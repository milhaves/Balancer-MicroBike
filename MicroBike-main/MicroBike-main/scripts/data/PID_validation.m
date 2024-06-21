clear all
close all
clc

g = 9.81;
c = .006;%1/4 inch trail
a = .023;
h = .1;
U = 1.5; %speed of microbike
lambda = 1.0;
b = 0.15;
s = tf('s');
G = minreal(sin(lambda)/b* (a/h*U*s + U^2/h - g*a*c/(h*U))/(s^2-g/h))
% Cstar = 1;%(s+.15+1.4j)*(s+.15-1.4j)/s;
Cstar = (s^2+20*s+60)/s;%kp2,ki6,kd0.1

wnservo = 50;
zservo = 1;
Gservo = (2*zservo*wnservo*s+wnservo^2)/(s^2+2*zservo*wnservo*s+wnservo^2);

Klocus = 0.1
Gcl = minreal(Klocus*Cstar*Gservo*G/(1+Klocus*Cstar*Gservo*G))

figure
step(Gcl,Gcl*1/(.5*s+1))
legend('no prefilter','prefilter')
xlabel('time (s)')
ylabel('roll')

[ysim,tsim] = step(Gcl*1/(.5*s+1));%*1/(.5*s+1));
ysim = ysim*0.15*180/pi;%step magnitude was 0.1 radian

startTime = 24.5;%19.75
% d = load('data_20240310-161430.txt');
% d = load('data_20240310-204116.txt');
%d = load('data_20240310-205654.txt');
%d = load('data_20240310-161430.txt');
d = load('good_data_20240327-203028.txt');

plot(d(:,1)-startTime,d(:,3),d(:,1)-startTime,d(:,4),tsim,ysim)
xlabel('Time (s)')
ylabel('Roll (deg)')
legend('command','actual','simulation')
xlim([0 2])
ylim([-2 15])

% figure()
% plot(d(:,1)-startTime,d(:,5))
% xlabel('Time (s)')
% ylabel('U (m/s)')