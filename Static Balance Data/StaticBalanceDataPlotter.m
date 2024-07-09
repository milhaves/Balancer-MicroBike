%% Static Balance Data plotter
clc; clear all; clf; close all;

data = readmatrix('webots_data.txt');

time = data(:,1);
roll = data(:,3);
lean = data(:,8);
goalLean = data(:,9);
lean_sim = data(:,10);

% subplot(2,1,1);
% plot(time,roll)
% ylabel('Theta 1 (rad)')
% xlim([0 1.01])

% subplot(2,1,2);
plot(time,lean,time,goalLean,'k.',time,lean_sim)
xlabel('Time (s)')
ylabel('Current/Desired Pos (Rad)')
legend('Current Lean','Goal Lean','LeanSim')
xlim([0 1.01])