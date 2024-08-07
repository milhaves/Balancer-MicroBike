clear all;
close all
clc;

%I do not know what this is supposed to be or why it's here...
% static = readmatrix('WithoutBalancer.txt');
% timeStatic = static(:,1);
% timeStatic = timeStatic-2;
% rollStatic = static(:,3);
% rollStatic = rollStatic/0.15;

%load webots data
webots = readmatrix('webots_data.txt');
timeWebots = webots(:,1);
timeWebots = timeWebots;
rollWebots = webots(:,3);
% rollWebots = rollWebots/-0.1+1;%WTF is this

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
z2 = 3;
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

Klocus = 0.35
Kd = Klocus
Kp = (z1+z2)*Klocus
Ki = z1*z2*Klocus

Gcl = minreal(Klocus*Cstar*Gservo*G/(1+Klocus*Cstar*Gservo*G))

[num,den] = tfdata(Gcl,'v');
real(roots(den))
plot(real(roots(den)),imag(roots(den)),'k*')
legend('no actuator','with actuator dynamics','cl eigs')

figure
step_mag = 0.15
tau_prefilt = 0.5; % we filter goal roll before sending to controller.
G_prefilt = 1/(tau_prefilt*s+1); %total prefilter TF
[y_lin,t_lin] = step(G_prefilt*Gcl); %simulate step
y_lin = y_lin*step_mag; %scale the step response based on webots experiment magnitude

plot(t_lin,y_lin,'k',timeWebots,rollWebots,'r')
xlabel('Time (s)')
ylabel('Roll angle (rad)')
title('microbike step response in roll with simulated actuator dynamics')
xlim([0 2])
% plot(timeWebots,rollWebots,'k.')
legend('Modeled Step','Webots Experiment')
