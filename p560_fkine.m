function    [T,Pos]=p560_fkine(theta)
% The function of PUMA560 forward kinematics
% input theta(1×6，degree) for T and Pos(xyz and rpy) as output
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
%% isLimit_theta
if (theta(1)<-160)|(theta(1)>160)
    fprintf("theta1 out of range!\n");
end
if (theta(2)< -45)|(theta(2)>225)
    fprintf("theta2 out of range!\n");
end
if (theta(3)<-225)|(theta(3)>45)
    fprintf("theta3 out of range!\n");
end
if (theta(4)<-110)|(theta(4)>170)
    fprintf("theta4 out of range!\n");
end
if (theta(5)<-100)|(theta(5)>100)
    fprintf("theta5 out of range!\n");
end
if (theta(6)<-266)|(theta(6)>266)
    fprintf("theta6 out of range!\n");
end

%% R01,R12,R23,R34,R45,R56; p23,p34;

    th1=theta(1);th2=theta(2);th3=theta(3); 
    th4=theta(4);th5=theta(5);th6=theta(6);
    
    R01=[cosd(th1),-sind(th1),0;
         sind(th1), cosd(th1),0;
                 0,         0,1];
    
    R12=[cosd(th2),-sind(th2), 0;
                 0,         0,-1;
         sind(th2), cosd(th2), 0];
   
    R23=[cosd(th3),-sind(th3),0;
         sind(th3), cosd(th3),0;
                 0,         0,1];
             
    R34=[ cosd(th4),-sind(th4), 0;
                  0,         0, 1;
         -sind(th4),-cosd(th4), 0];
     
    R45=[cosd(th5),-sind(th5), 0;
                 0,         0,-1;
         sind(th5), cosd(th5), 0];
     
    R56=[cosd(th6),-sind(th6), 0;
                 0,         0, 1;
        -sind(th6),-cosd(th6), 0];
    
    p23=[0.4318 0       0.15005 ]';
    p34=[0.0203 0.4318  0       ]';
    
    %% 求末端位置和姿态
    p24=p23+R23*p34;
    p=R01*R12*p24;
    R=R01*R12*R23*R34*R45*R56;
    rpy=[0 0 0];
    if (R(3,2)==0)&&(R(3,3)==0) % beta=±90
        if R(3,1)>0  %beta=-90
             rpy(1)=0;
             rpy(2)=-pi/2;
             rpy(3)=-atan2(R(1,2),R(2,2));
        elseif R(3,1)<0  %beta=90
             rpy(1)=0;
             rpy(2)=pi/2;
             rpy(3)=atan2(R(1,2),R(2,2));
        else
            fprintf("error!\n");
            return;
        end
    else
        rpy(2)=atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
        rpy(1)=atan2(R(2,1)/cos(rpy(2)),R(1,1)/cos(rpy(2)));
        rpy(3)=atan2(R(3,2)/cos(rpy(2)),R(3,3)/cos(rpy(2)));
    end
    RPY=rpy*180/pi;
    %% 返回值
    T=[   R   ,p ;
       [0 0 0],1 ];
    Pos=[p' RPY];