function [P]= traj_plan_line(P1,P2,N)
%[P]= traj_plan_line(P1,P2,N) 直线轨迹规划
%   参数：起点P1，终点P2的广义坐标
%         插值点的个数N
%   结果：线性插值序列P
% 
Del=(P2-P1)/(N+1);
P=P1;
for i=1:N+1
    P=[P;P1+i*Del];
end

