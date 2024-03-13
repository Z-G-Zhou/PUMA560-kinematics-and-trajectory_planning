%% 计算直线插补序列并画图展示
clear,clc,close all;
P1xyz=[0.452,  -0.150,  0.432];
P2xyz=[0.495,   0.082,  0.588];
P3xyz=[0.253,   0.361,  0.229];
N1=48; N2=99; % 插值点的个数 
P= traj_plan_circle(P1xyz,P2xyz,P3xyz,[N1,N2]);
P(N1+N2+3,6)=0;

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

%% 图：末端轨迹图，q-t图, qd-t图, qdd-t图

%%%%%%%%%%%%%%%%%%%%%%%% 末端轨迹图 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
plot3(P(:,1),P(:,2),P(:,3),'.');
grid on;hold on;
plot3(P1xyz(1),P1xyz(2),P1xyz(3),'o','color','m');
plot3(P2xyz(1),P2xyz(2),P2xyz(3),'o','color','c');
plot3(P3xyz(1),P3xyz(2),P3xyz(3),'o','color','g');
mdl_puma560;
T=p560.fkine(th/180*pi);
pos_xyz=transl(T);
plot3(pos_xyz(:,1),pos_xyz(:,2),pos_xyz(:,3));
hold on;
p560.teach(th*pi/180);

%%%%%%%%%%%%%%%%%%%%%%%%  q-t图 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t=linspace(0,10,N1+N2+3); % 时间序列
figure;
subplot(3,2,1), plot(t,th(:,1),'r'),xlabel('t'),ylabel('q1');grid on; 
subplot(3,2,2), plot(t,th(:,2),'r'),xlabel('t'),ylabel('q2');grid on; 
subplot(3,2,3), plot(t,th(:,3),'r'),xlabel('t'),ylabel('q3');grid on; 
subplot(3,2,4), plot(t,th(:,4),'r'),xlabel('t'),ylabel('q4');grid on; xlim([0 10]);ylim([-5 10])
subplot(3,2,5), plot(t,th(:,5),'r'),xlabel('t'),ylabel('q5');grid on; 
subplot(3,2,6), plot(t,th(:,6),'r'),xlabel('t'),ylabel('q6');grid on; 

%%%%%%%%%%%%%%%%%%%%%%%%  qd-t图 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
subplot(3,2,1), plot(t(1:end-1),diff(th(:,1))./diff(t),'g'),xlabel('t'),ylabel('qd1');grid on; 
subplot(3,2,2), plot(t(1:end-1),diff(th(:,2))./diff(t),'g'),xlabel('t'),ylabel('qd2');grid on; 
subplot(3,2,3), plot(t(1:end-1),diff(th(:,3))./diff(t),'g'),xlabel('t'),ylabel('qd3');grid on; 
subplot(3,2,4), plot(t(1:end-1),diff(th(:,4))./diff(t),'g'),xlabel('t'),ylabel('qd4');grid on; xlim([0 10]);ylim([-5 10])
subplot(3,2,5), plot(t(1:end-1),diff(th(:,5))./diff(t),'g'),xlabel('t'),ylabel('qd5');grid on; 
subplot(3,2,6), plot(t(1:end-1),diff(th(:,6))./diff(t),'g'),xlabel('t'),ylabel('qd6');grid on; 

%%%%%%%%%%%%%%%%%%%%%%%%  qdd-t图 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
subplot(3,2,1), plot(t(1:end-2),diff(diff(th(:,1))./diff(t))./diff(t),'b'),xlabel('t'),ylabel('qdd1');grid on; 
subplot(3,2,2), plot(t(1:end-2),diff(diff(th(:,2))./diff(t))./diff(t),'b'),xlabel('t'),ylabel('qdd2');grid on; 
subplot(3,2,3), plot(t(1:end-2),diff(diff(th(:,3))./diff(t))./diff(t),'b'),xlabel('t'),ylabel('qdd3');grid on; 
subplot(3,2,4), plot(t(1:end-2),diff(diff(th(:,4))./diff(t))./diff(t),'b'),xlabel('t'),ylabel('qdd4');grid on; xlim([0 10]);ylim([-5 10])
subplot(3,2,5), plot(t(1:end-2),diff(diff(th(:,5))./diff(t))./diff(t),'b'),xlabel('t'),ylabel('qdd5');grid on; 
subplot(3,2,6), plot(t(1:end-2),diff(diff(th(:,6))./diff(t))./diff(t),'b'),xlabel('t'),ylabel('qdd6');grid on; 
