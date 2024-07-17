%% Step Roll With Balancer Data plotter
clc; clear all; clf; close all;

data = readmatrix('webots_data.txt');

time = data(:,1);
roll_desired = data(:,2);
roll = data(:,3);
steer_desired = data(:,4);
steer = data(:,5);
lean_desired = data(:,6);
lean = data(:,7);

figure
plot(time, roll_desired,"k.",time,roll,"b.")
title('Desired Roll Angle vs Actual')
xlabel('Time (s)')
ylabel('Roll (rad)')
legend('Roll Desired','Roll')

figure
plot(time,steer_desired,"k.",time,steer,"b.")
title('Desired Steer Angle vs Actual')
xlabel('Time (s)')
ylabel('Steer Angle (rad)')
legend('Steer Desired','Steer')

figure
plot(time,lean_desired,"k.",time,lean,"b.")
title('Desired Lean Angle vs Actual')
xlabel('Time (s)')
ylabel('Lean Angle (rad)')
legend('Lean Desired','Lean')