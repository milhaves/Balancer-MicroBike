clear all;
close all
clc;

g = 9.81;
c = .01
a = .05;
h = .06;
U = 1.0; %speed of microbike
lambda = 1.0;
b = 0.15;
s = tf('s');
G = minreal(sin(lambda)/b* (a/h*U*s + U^2/h - g*a*c/(h*U))/(s^2-g/h))

rlocus(G)