%% Goal Roll Plotter
clc; clear all; clf; close all;

active = readmatrix('WithBalancer.txt');
static = readmatrix('WithoutBalancer.txt');

timeActive = active(:,1);
% goalRoll = active(:,2);
rollActive = active(:,3);

timeStatic = static(:,1);
goalRoll = static(:,2);
rollStatic = static(:,3);
leanStatic = static(:,8);

figure
plot(timeStatic,goalRoll,'k-',timeActive,rollActive,'b.',timeStatic,rollStatic,'r.')
xlabel('Time (s)')
ylabel('Theta 1 (rad)')
legend('Goal Roll','With Balancer', 'Without Balancer')
title('Step Roll Comparison')

figure
plot(timeStatic,leanStatic,'b.')
xlabel('Time (s)')
ylabel('Theta 2 (rad)')