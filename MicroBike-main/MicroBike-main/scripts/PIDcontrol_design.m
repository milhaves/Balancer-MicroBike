clear all;
close all
clc;

g = 9.81;
c = .01;
a = .05;
h = .06;
U = 1.5; %speed of microbike
lambda = 1.0;
b = 0.15;
s = tf('s');
G = minreal(sin(lambda)/b* (a/h*U*s + U^2/h - g*a*c/(h*U))/(s^2-g/h))
% Cstar = 1;%(s+.15+1.4j)*(s+.15-1.4j)/s;
z1 = 5;
z2 = 8;
Cstar = (s+z1)*(s+z2)/s

% Cstar = (s+25)*(s+20)/s
% rlocus(G*Cstar)
% hold on
wnservo = 23;
zservo = 1;
Gservo = wnservo^2/(s^2+2*zservo*wnservo*s+wnservo^2);

figure
rlocus(G*Cstar,G*Cstar*Gservo)

Klocus = 0.15
Kd = Klocus
Kp = (z1+z2)*Klocus
Ki = z1*z2*Klocus

Gcl = minreal(Klocus*Cstar*Gservo*G/(1+Klocus*Cstar*Gservo*G))
figure
step(Gcl*1/(.25*s+1))
