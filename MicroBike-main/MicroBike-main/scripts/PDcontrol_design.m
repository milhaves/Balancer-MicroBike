clear all;
close all
clc;

g = 9.81;
c = .006;%1/4 inch trail
a = .023;
h = .1;
U = 1.7; %speed of microbike
lambda = 1.0;
b = 0.15;
s = tf('s');
G = minreal(sin(lambda)/b* (a/h*U*s + U^2/h - g*a*c/(h*U))/(s^2-g/h))
% Cstar = 1;%(s+.15+1.4j)*(s+.15-1.4j)/s;
Cstar = (s+20);

% Cstar = (s+25)*(s+20)/s
% rlocus(G*Cstar)
% hold on
wnservo = 23;
zservo = 1;
Gservo = (2*zservo*wnservo*s+wnservo^2)/(s^2+2*zservo*wnservo*s+wnservo^2);

figure
rlocus(G*Cstar,G*Cstar*Gservo)
hold on

Klocus = 0.1
Gcl = minreal(Klocus*Cstar*Gservo*G/(1+Klocus*Cstar*Gservo*G))
[num,den] = tfdata(Gcl,'v')
rootsre = real(roots(den))
rootsim = imag(roots(den))
plot(rootsre,rootsim,'r*')




figure
step(Gcl,Gcl*1/(.25*s+1))
legend('no prefilter','prefilter')
xlabel('time (s)')
ylabel('roll')
