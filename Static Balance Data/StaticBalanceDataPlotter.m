%% Static Balance Data plotter
clc; clear all; clf; close all;

data = readmatrix('webots_data.txt');

time = data(:,1);
roll = data(:,3);
lean = data(:,8);
goalLean = data(:,9);
lean_sim = data(:,10);
leanrate = data(:,11);

goalRoll = zeros(size(roll));

subplot(2,1,1);
plot(time,roll,'b.',time,goalRoll,'k-')
ylabel('Theta 1 (rad)')
xlim([0 .25])

subplot(2,1,2);
% plot(time,lean,'b.',time,goalLean,'k.',time,lean_sim,'r.')
plot(time,lean,'b.',time,lean_sim,'r.')
ylabel('Current/Desired Pos (Rad)')
% legend('Current Lean','Goal Lean','LeanSim')
legend('Current Lean','LeanSim')
xlim([0 .25])
% subplot(3,1,3);
% plot(time, leanrate,'k.',time,goalLean,'b.')
xlabel('Time (s)')