function [qc]=cartesian_space_circle_function(pos1,pos2,pos3,t,fps)
%[qc]=cartesian_space_circle_function(pos1,pos2,pos3,fps,t) 笛卡尔圆弧轨迹规划
%   参数：起点pos1、中间点pos2、终点pos3 的三组位姿pos=[x,y,z,r,p,y]
%         1->2,2->3 的时间t=[t1,t2],帧率fps：插值点N=fps*t
%   结果：N+3组关节角序列qc
% 
%% 计算直线插补序列并画图展示
P1xyz=pos1(1:3);P1rpy=pos1(4:6);
P2xyz=pos2(1:3);P2rpy=pos2(4:6);
P3xyz=pos3(1:3);P3rpy=pos3(4:6);
% P1xyz=[0.452,  -0.150,  0.432];
% P2xyz=[0.495,   0.082,  0.588];
% P3xyz=[0.253,   0.361,  0.229];
N1=t(1)*fps; N2=t(2)*fps; % 插值点的个数 
Pxyz= traj_plan_circle(P1xyz,P2xyz,P3xyz,[N1,N2]);% 得到位置xyz序列

Del1=(P2rpy-P1rpy)/(N1+1);
P12=P1rpy;
for i=1:N1+1
    P12=[P12;P1rpy+i*Del1];
end
Del2=(P3rpy-P2rpy)/(N2+1);
P23=P2rpy;
for i=1:N2+1
    P23=[P23;P2rpy+i*Del2];
end
Prpy=[P12(1:end-1,:);P23];  %得到姿态rpy序列
P=[Pxyz,Prpy];  %得到pos序列

%% 全部P点逆运动学的求解和选择
th(N1+N2+3,6)=0;% 六关节角解序列初始化
thi_1=[0 0 0 0 0 0]; % 每个循环用于判据对比的thi_1的初始化

%%%%%%%%%%%%%% 设置选择时的权重系数a %%%%%%%%%%%%%%%%%%
a=[2,2,2,1,1,1]; %设置权重系数
%%%%%%%%%%%%%% 设置选择时的权重系数a %%%%%%%%%%%%%%%%%%

for i=1:N1+N2+3
    Pi=P(i,:); % 某一点Pi的pos=[x,y,z,r,p,y]
    [thi_all,thi_true]=p560_ikine(Pi);% 逆运动学求解
    
    %%%%%%%%%%%%%% 在thi_true中挑选最优的thi %%%%%%%%%%%%%%%%%%
    [m,n]=size(thi_true); % 矩阵的行列数 m，n
    best_m=1;  % 最优的一组解所在的行数
    for j=1:m % 对每一行进行判断（一行为一组解）
        thj=thi_true(j,     :); %第i的点Pi对应的第j个真实解
        thm=thi_true(best_m,:); %第i的点Pi的当前最优解
        
        % 最优判据关系式  a=[ , , , , , ]为六个权重系数
        para_j=a(1)*abs(thj(1)-thi_1(1))+a(2)*abs(thj(2)-thi_1(2))+a(3)*abs(thj(3)-thi_1(3))+a(4)*abs(thj(4)-thi_1(4))+a(5)*abs(thj(5)-thi_1(5))+a(6)*abs(thj(6)-thi_1(6)) ; 
        para_m=a(1)*abs(thm(1)-thi_1(1))+a(2)*abs(thm(2)-thi_1(2))+a(3)*abs(thm(3)-thi_1(3))+a(4)*abs(thm(4)-thi_1(4))+a(5)*abs(thm(5)-thi_1(5))+a(6)*abs(thm(6)-thi_1(6)) ; 
        
        
        if para_j < para_m
            best_m=j;
        end
    end
    thi=thi_true(best_m,:);
    
    %%%%%%%%%%%%%% 得到的thi填入th %%%%%%%%%%%%%%%%%%
    th(i,:)=thi;
    thi_1=thi; %下一循环的thi_1
end
qc=th;
