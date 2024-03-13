clear;clc;close all;
%% 起末点位姿，关节角
pos1=[0.242,  -0.676,  0.137,   -65.4,  20.7,  37.6];
pos2=[0.299,  -0.026,  0.596,  -155.7,  -6.3,  36.5];
[th_all_1,th_true_1]=p560_ikine(pos1);
[th_all_2,th_true_2]=p560_ikine(pos2);
th1=th_true_1(1,:);
th2=th_true_2(1,:);
%% 每个关节角五次多项式插补轨迹点
[qq1,vv1,aa1,tt] = traj_plan_5([th1(1),th2(1)],[0 0],[0 0],[0 5],48);
[qq2,vv2,aa2   ] = traj_plan_5([th1(2),th2(2)],[0 0],[0 0],[0 5],48);
[qq3,vv3,aa3   ] = traj_plan_5([th1(3),th2(3)],[0 0],[0 0],[0 5],48);
[qq4,vv4,aa4   ] = traj_plan_5([th1(4),th2(4)],[0 0],[0 0],[0 5],48);
[qq5,vv5,aa5   ] = traj_plan_5([th1(5),th2(5)],[0 0],[0 0],[0 5],48);
[qq6,vv6,aa6   ] = traj_plan_5([th1(6),th2(6)],[0 0],[0 0],[0 5],48);
qq=[qq1;qq2;qq3;qq4;qq5;qq6];   %  六关节角的位置序列 
vv=[vv1;vv2;vv3;vv4;vv5;vv6];   %  六关节角的速度序列 
aa=[aa1;aa2;aa3;aa4;aa5;aa6];   %  六关节角的加速度序列 
tt;                             %  时间序列 
q=qq'; v=vv';a=aa';t=tt';       %求转置
%% 求正解，绘制末端位置
mdl_puma560; % rtb工具箱建模puma560
T=p560.fkine(q*pi/180); %求正解，得到每次对应的空间位姿矩阵
pos_xyz=transl(T);   %空间位姿矩阵求位置矩阵

plot3(pos_xyz(:,1), pos_xyz(:,2),pos_xyz(:,3),'r');
%W=[-1000,+1000,-1000,+1000,-1000,+1000]; axis(W); %设置坐标轴范围
hold on;grid on;
plot3(pos1(1),pos1(2),pos1(3),'o','color','m');
plot3(pos2(1),pos2(2),pos2(3),'o','color','c');
p560.plot(q/180*pi);

%% 绘图六关节的 q v a 图
figure;
subplot(3,2,1), plot(tt,qq(1,:)),xlabel('t'),ylabel('q1');grid on;
subplot(3,2,2), plot(tt,qq(2,:)),xlabel('t'),ylabel('q2');grid on;
subplot(3,2,3), plot(tt,qq(3,:)),xlabel('t'),ylabel('q3');grid on;
subplot(3,2,4), plot(tt,qq(4,:)),xlabel('t'),ylabel('q4');grid on;
subplot(3,2,5), plot(tt,qq(5,:)),xlabel('t'),ylabel('q5');grid on;
subplot(3,2,6), plot(tt,qq(6,:)),xlabel('t'),ylabel('q6');grid on;

figure;
subplot(3,2,1), plot(tt,vv(1,:)),xlabel('t'),ylabel('qd1');grid on;
subplot(3,2,2), plot(tt,vv(2,:)),xlabel('t'),ylabel('qd2');grid on;
subplot(3,2,3), plot(tt,vv(3,:)),xlabel('t'),ylabel('qd3');grid on;
subplot(3,2,4), plot(tt,vv(4,:)),xlabel('t'),ylabel('qd4');grid on;
subplot(3,2,5), plot(tt,vv(5,:)),xlabel('t'),ylabel('qd5');grid on;
subplot(3,2,6), plot(tt,vv(6,:)),xlabel('t'),ylabel('qd6');grid on;

figure;
subplot(3,2,1), plot(tt,aa(1,:)),xlabel('t'),ylabel('qdd1');grid on;
subplot(3,2,2), plot(tt,aa(2,:)),xlabel('t'),ylabel('qdd2');grid on;
subplot(3,2,3), plot(tt,aa(3,:)),xlabel('t'),ylabel('qdd3');grid on;
subplot(3,2,4), plot(tt,aa(4,:)),xlabel('t'),ylabel('qdd4');grid on;
subplot(3,2,5), plot(tt,aa(5,:)),xlabel('t'),ylabel('qdd5');grid on;
subplot(3,2,6), plot(tt,aa(6,:)),xlabel('t'),ylabel('qdd6');grid on;
