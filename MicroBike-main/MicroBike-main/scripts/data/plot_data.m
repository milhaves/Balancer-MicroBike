clear all
close all
clc

startTime1 = 23;
startTime2 = 332;
% d = load('data_20240310-161430.txt');
%d = load('data_20240310-204116.txt');
%d = load('data_20240326-213649.txt');
%d=load('data_20240326-220742.txt');
d1=load('good_data_20240327-203028.txt');
d2=load('stiffer_fork_kp2.5_ki8_kd015_data_20240326-220742.txt');
%time1=d1(:,1);

% time2=d2(:,1);
% roll1=d1(:,4);
% roll2=d2(:,4);
% rollcommend1=d1(:,3);
% rollcommend2=d2(:,3);
plot(d1(:,1)-startTime1,d1(:,3),d1(:,1)-startTime1,d1(:,4));
xlabel('Time (s)')
ylabel('Roll (deg)')
legend('command','actual')
% hold on
% 
% plot(d2(:,1)-startTime2,d2(:,3),d2(:,1)-startTime2,d2(:,4));
% xlabel('Time (s)')
% ylabel('Roll (deg)')
% legend('command','actual')
xlim([0 4])
%ylim([-20 20])
