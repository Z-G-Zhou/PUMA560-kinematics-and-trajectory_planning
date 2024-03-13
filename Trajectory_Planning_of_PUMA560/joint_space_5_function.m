function [qj]=joint_space_5_function(pos1,pos2,t,fps)
%[qj]=joint_space_5_function(pos1,pos2,t,fps) 关节空间轨迹规划
%   方式：从pos1静止开始，运动到pos2停止
%   参数：起点位姿pos1,终点位姿pos2,
%         运行时间t , 动作帧率fps：插值点的个数N=t*fps
%   结果：角度序列qj
% 
N=t*fps;
[th_all_1,th_true_1]=p560_ikine(pos1);
[th_all_2,th_true_2]=p560_ikine(pos2);
th1=th_true_1(1,:); 
th2=th_true_2(1,:); % 此处任意选取第一组解为起止点的关节角的解

[qq1,vv1,aa1,tt] = traj_plan_5([th1(1),th2(1)],[0 0],[0 0],[0 t],N);
[qq2,vv2,aa2   ] = traj_plan_5([th1(2),th2(2)],[0 0],[0 0],[0 t],N);
[qq3,vv3,aa3   ] = traj_plan_5([th1(3),th2(3)],[0 0],[0 0],[0 t],N);
[qq4,vv4,aa4   ] = traj_plan_5([th1(4),th2(4)],[0 0],[0 0],[0 t],N);
[qq5,vv5,aa5   ] = traj_plan_5([th1(5),th2(5)],[0 0],[0 0],[0 t],N);
[qq6,vv6,aa6   ] = traj_plan_5([th1(6),th2(6)],[0 0],[0 0],[0 t],N);
qq=[qq1;qq2;qq3;qq4;qq5;qq6];   %  六关节角的位置序列 
% vv=[vv1;vv2;vv3;vv4;vv5;vv6];   %  六关节角的速度序列 
% aa=[aa1;aa2;aa3;aa4;aa5;aa6];   %  六关节角的加速度序列 
% tt;                             %  时间序列 
% q=qq'; v=vv';a=aa';t=tt';       %求转置
qj=qq';