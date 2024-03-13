clear,clc,close all;
%% 分别求解自编函数和rtb的正解结果
mdl_puma560;
thetad=[-30.0000  -58.2807   30.0000  -61.7193  -30.0000  -30.0000];%自定角度
theta=thetad/180*pi;   %转为弧度
[Tmine,Posmine]=p560_fkine(thetad); %自编函数求T和Pos：xyz+rpy
Trtb=p560.fkine(theta);  %工具箱求T
%% 验证T是否一致
Tmine
Trtb 
%% 验证Pos是否一致
Posmine                 %输出自编函数求解的Pos
p560.teach(theta)       %显示rtb的得到的pos和当前机器人姿态