clear;clc;close all;
q_array=[30,60,40]; %指定参数：位置
v_array=[20,30,20];%指定参数：速度
t_array=[0,2,4];   %指定参数：时间
N=[98 98];%指定参数：插值个数

[qq,vv,aa,tt] = traj_plan_3(q_array,v_array,t_array,N);

subplot(3,1,1),plot(tt,qq,'r'),xlabel('t'),ylabel('q');hold on;
plot(t_array,q_array,'o','color','g'),grid on;
subplot(3,1,2),plot(tt,vv,'b'),xlabel('t'),ylabel('v');hold on;
plot(t_array,v_array,'*','color','y'),grid on;
subplot(3,1,3),plot(tt,aa,'g'),xlabel('t'),ylabel('a');hold on;
plot(t_array,a_array,'^','color','r'),grid on;