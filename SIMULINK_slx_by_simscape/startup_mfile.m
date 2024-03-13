clear; clc;close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%% joint_space %%%%%%%%%%%%%%%%%%%%%%%%%%%
P1j=[0.242,  -0.676,  0.137,   -65.4,  20.7,  37.6];
P2j=[0.299,  -0.026,  0.596,  -155.7,  -6.3,  36.5];

fpsj=10;tj1=6;  % 帧率和时间

[qj]=joint_space_5_function(P1j,P2j,tj1,fpsj);
tj=linspace(0,6,fpsj*tj1+2);
%%%%%%%%%%%%%%%%%%%%%% cartesian_space_circle %%%%%%%%%%%%%%%%%%%%%%%%%%%
%指定圆弧上的三点，运动方向为P1c --> P2c --> P3c。 
P1c=[0.452,  -0.150,  0.432, 0, 0, 0]; 
P2c=[0.495,   0.082,  0.588, 0, 0, 0];
P3c=[0.253,   0.361,  0.229, 0, 0, 0];

fpsc=10;tc1=4;tc2=6;  % 帧率和时间

% 求解关节角序列，共  fps*(t1+t2)+3  组 。  xyz 为圆弧插值 ， rpy 为线性插值
[qc]=cartesian_space_circle_function(P1c,P2c,P3c,[tc1,tc2],fpsc);% qc：得到的圆弧轨迹关节角序列
tc=linspace(0,tc1+tc2,fpsc*(tc1+tc2)+3);% tc：得到的圆弧轨迹时间序列

save data_for_sims tj tc qj qc % 将sims用到的数据存下来
clear;clc;close all;
load('data_for_sims.mat')  % 将sims用到的数据加载
