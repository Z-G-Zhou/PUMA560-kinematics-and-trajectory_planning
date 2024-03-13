function [thetaall,thetatrue]=p560_ikine(Pos)
% The function of PUMA560 forward kinematics
% output theta(1×6，degree) by Pos(xyz and rpy_degree) as input
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         The MDH parameters of PUMA 560
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% |  number  | alpha  |   a   |   d   | theta  | min | max |
% |     1    |   90   |     0 |   0   | theta1 |-160 | 160 |
% |     2    |   0    | 0.4318|   0   | theta2 | -45 | 225 |
% |     3    |  -90   | 0.0203|0.15005| theta3 |-225 |  45 |
% |     4    |   90   |     0 | 0.4318| theta4 |-110 | 170 |
% |     5    |  -90   |     0 |   0   | theta5 |-100 | 100 |
% |     6    |    0   |     0 |   0   | theta6 |-266 | 266 |
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% 操作臂参数的初始化：DH参数、关节角范围和中间参数的设置
x=Pos(1);y=Pos(2);z=Pos(3);roll=Pos(4);pitch=Pos(5);yaw=Pos(6);
thm=[-160 -45 -225 -110 -100 -266;
      160 225   45  170  100  266]; % 关节角范围，第一行为最小值，第二行为最大值
r=x^2+y^2+z^2;

%% 求解theta_3
Del3=-4*r^2+3.16664*r-0.069; 
a=r-0.3783;b=0.746;c=r-0.41336;
if (Del3>=0)
    t3(1,1)=(-b+sqrt(Del3))/(2*a);t3(2,1)=(-b-sqrt(Del3))/(2*a);
    t3(1,2)=t3(1,1);t3(2,2)=t3(2,1);
else
    fprintf("tan(theta3/2)方程根判别式小于零，theta3无解！\n");
    fprintf("r=x^2+y^2+z^2需在区间[0.0224,0.7692]内！\n");
    return;
end
th3=2.*atan(t3).*180./pi; % th3是一个2×2矩阵，同行元素相同，不同行代表不同的两个theta3的解
%% 求解theta2
f1=0.0203.*cosd(th3)-0.4318.*sind(th3)+0.4318;
f2=0.0203.*sind(th3)+0.4318.*cosd(th3);
f3=0.15005.*ones(2);
Del2=4.*f1.^2-4.*(z^2*ones(2)-f2.^2);
a=z+f2;b=-2.*f1;c=z-f2;
if (Del2>=0)
    t2(:,1)=(-b(:,1)+sqrt(Del2(:,1)))./(2*a(:,1));
    t2(:,2)=(-b(:,2)-sqrt(Del2(:,2)))./(2*a(:,2));
else
    fprintf("tan(theta2/2)方程根判别式小于零，theta2无解！\n");
    return;
end
th2=2.*atan(t2).*180./pi;  % th2是一个2×2矩阵,行标对应theta3，列表对应所在行theta3相应的theta2
                 % 如th2（1，2）theta3的第一个解的第二个theta2的解
%% 求解theta1
g1=f1.*cosd(th2)-f2.*sind(th2);
g2=-f3;                     % g1,g2是2×2矩阵。代表第i个theta3的第j个theta2确定应的g
                            % 如g1（1，2）是theta3的第一个解和其对应的第二个theta2的解确定的g1
angle1=180*atan2(y,x)/pi;  %(x,y)的角度angle1
angle2=180.*atan2(g2,g1)./pi;  % (g1,g2)的角度angle2 
th1=angle1-angle2;
if th1<-180
    th1=th1+360;
elseif th1>180
    th1=th1-360;
end
%% 求解theta4,theta5,theta6
R06=rotz(roll)*roty(pitch)*rotx(yaw);
thetaall(8,6)=0;  % 全部逆解矩阵的初始化
for i=1:4   % 分别将四组theta123赋值给循环变量sth123，循环四次，计算th456,并赋值全部逆解矩阵theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 分别将四组theta123赋值给循环变量sth123  %%%%%%%%%%%%%%%%%%%%%%%%%%
    if i==1   
        sth1=th1(1,1);
        sth2=th2(1,1);
        sth3=th3(1,1);
    elseif i==2
        sth1=th1(1,2);
        sth2=th2(1,2);
        sth3=th3(1,2);
    elseif i==3
        sth1=th1(2,1);
        sth2=th2(2,1);
        sth3=th3(2,1);
    elseif i==4
        sth1=th1(2,2);
        sth2=th2(2,2);
        sth3=th3(2,2);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%% 计算R4ORG6 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    R01=[cosd(sth1),-sind(sth1),0;
         sind(sth1), cosd(sth1),0;
                  0,          0,1];

    R12=[cosd(sth2),-sind(sth2), 0;
                  0,          0,-1;
         sind(sth2), cosd(sth2), 0];

    R23=[cosd(sth3),-sind(sth3),0;
         sind(sth3), cosd(sth3),0;
                  0,          0,1];
    
    R34ORG=[   1,   0,  0;
               0,   0,  1;
               0,  -1,  0];
    
    R04ORG=R01*R12*R23*R34ORG;
    R4ORG6=R04ORG\R06;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 计算sth456 %%%%%%%%%%%%%%%%%%%%%%%%%%
    if R4ORG6(3,1)==0&&R4ORG6(3,2)==0 % beta=0或180
        sth4(1)=0;sth5(1)=  0;sth6(1)=180*atan2(-R4ORG6(1,2),R4ORG6(1,1))/pi;
        sth4(2)=0;sth5(2)=180;sth6(2)=180*atan2(R4ORG6(1,2),-R4ORG6(1,1))/pi;
    else
        sth5(1)=-atan2(sqrt(R4ORG6(3,1)^2+R4ORG6(3,2)^2),R4ORG6(3,3))*180/pi;
        sth4(1)=atan2(R4ORG6(2,3)/sind(-sth5(1)),R4ORG6(1,3)/sind(-sth5(1)))*180/pi;
        sth6(1)=atan2(R4ORG6(3,2)/sind(-sth5(1)),-R4ORG6(3,1)/sind(-sth5(1)))*180/pi;
        
        sth5(2)=-atan2(-sqrt(R4ORG6(3,1)^2+R4ORG6(3,2)^2),R4ORG6(3,3))*180/pi;
        sth4(2)=atan2(R4ORG6(2,3)/sind(-sth5(2)),R4ORG6(1,3)/sind(-sth5(2)))*180/pi;
        sth6(2)=atan2(R4ORG6(3,2)/sind(-sth5(2)),-R4ORG6(3,1)/sind(-sth5(2)))*180/pi;
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%% theta %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    thetaall(2*i-1,:)=[sth1,sth2,sth3,sth4(1),sth5(1),sth6(1)];
    thetaall(2*i  ,:)=[sth1,sth2,sth3,sth4(2),sth5(2),sth6(2)];
end

%% 返回结果
thetatrue=thetaall; % 剔除重复解和超出关节范围的解后的解矩阵
epss=0.1; %设置关节角度重复剔除的精度
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 剔除重复解 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if abs(th3(1,1)-th3(2,1))<epss % th3重解
    thetatrue(5:8,:)=[];
    if abs(th2(1,1)-th2(1,2))<epss % th3重解th2重解
        thetatrue(3:4,:)=[];
    end
else                         %  th3不重解
    if abs(th2(2,1)-th2(2,2))<epss % 第二个th3的th2重解
        thetatrue(7:8,:)=[];         
    end
    if abs(th2(1,1)-th2(1,2))<epss % 第一个th3的th2重解
        thetatrue(3:4,:)=[]; 
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 剔除超出关节限制的解 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[m,n]=size(thetatrue); % thetatrue还剩m行
for j=m:-1:1  % 从下到上，逐行检验
    for k=1:6
        if (thetatrue(j,k)<thm(1,k))||(thetatrue(j,k)>thm(2,k))
            thetatrue(j,:)=[];
            break;
        end
    end
end
