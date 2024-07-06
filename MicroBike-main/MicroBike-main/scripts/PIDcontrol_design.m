clear all;
close all
clc;

g = 9.81;
c = .01;
a = .05;
hbike = .04;
hbal = .15;
mbike = 0.2;
mbal = 0.05;
hNew = (mbike*hbike+mbal*hbal)/(mbike+mbal)
U = 1.5; %speed of microbike
lambda = 1.0;
b = 0.15;
s = tf('s');
G = minreal(sin(lambda)/b* (a/hNew*U*s + U^2/hNew - g*a*c/(hNew*U))/(s^2-g/hNew))
% Cstar = 1;%(s+.15+1.4j)*(s+.15-1.4j)/s;
z1 = 3;
z2 = 8;
Cstar = (s+z1)*(s+z2)/s

% Cstar = (s+25)*(s+20)/s
figure
rlocus(G*Cstar)
% hold on
wnservo = 23;
zservo = 1;
Gservo = wnservo^2/(s^2+2*zservo*wnservo*s+wnservo^2);

figure
rlocus(G*Cstar,G*Cstar*Gservo)
hold on

Klocus = 0.22
Kd = Klocus
Kp = (z1+z2)*Klocus
Ki = z1*z2*Klocus

Gcl = minreal(Klocus*Cstar*Gservo*G/(1+Klocus*Cstar*Gservo*G))

[num,den] = tfdata(Gcl,'v');
real(roots(den))
plot(real(roots(den)),imag(roots(den)),'k*')
legend('no actuator','with actuator dynamics','cl eigs')

figure
step(Gcl*1/(.25*s+1))
