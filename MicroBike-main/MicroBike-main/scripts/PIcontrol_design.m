clear all;
close all
clc;

g = 9.81;
c = .01;
a = .05;
h = .06;
U = 1.0; %speed of microbike
lambda = 1.0;
b = 0.15;
s = tf('s');
G = minreal(sin(lambda)/b* (a/h*U*s + U^2/h - g*a*c/(h*U))/(s^2-g/h))
Cstar = (s+5)/s;
% rlocus(G*Cstar)
% hold on

U2 = 1.7;
G2 = minreal(sin(lambda)/b* (a/h*U2*s + U2^2/h - g*a*c/(h*U2))/(s^2-g/h))
rlocus(G*Cstar,G2*Cstar);


wnservo = 1600;
zservo = 1;
Gservo = wnservo^2/(s^2+2*zservo*wnservo*s+wnservo^2);

figure
rlocus(G*Cstar,G*Cstar*Gservo)
