clear; clc;
t=[0,2,4];   %指定起止时间
q=[30,60,40];%指定起止位置
v=[20,30,20];%指定起止速度
a=[2,4,2];   %指定起止加速度
N=[98,98];

[qq,vv,aa,tt] = traj_plan_5(q,v,a,t,N);

subplot(3,1,1),plot(tt,qq,'r'),xlabel('t'),ylabel('position');hold on;
plot(t,q,'o','color','g'),grid on;
subplot(3,1,2),plot(tt,vv,'b'),xlabel('t'),ylabel('velocity');hold on;
plot(t,v,'*','color','y'),grid on;
subplot(3,1,3),plot(tt,aa,'g'),xlabel('t'),ylabel('accelerate');hold on;
plot(t,a,'^','color','r'),grid on;