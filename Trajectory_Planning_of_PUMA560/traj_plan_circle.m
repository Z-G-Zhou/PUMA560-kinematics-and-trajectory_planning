function [P]= traj_plan_circle(P1,P2,P3,N)
%[P]= traj_plan_circle(P1,P2,P3,N) 圆弧轨迹规划
%   参数：起点P1，中间点P2，终点P3的三维坐标
%         插值点的个数N(二维行向量)
%   结果：圆弧点序列P
% 
%% 求圆心坐标和圆半径
u1=P2-P1;
w1=cross((P3-P1),u1);
u=u1/norm(u1);
w=w1/norm(w1);
v=cross(w,u);
bx=(P2-P1)*u';
cx=(P3-P1)*u';
cy=(P3-P1)*v';
h=(cx^2+cy^2-bx*cx)/(2*cy);
P0=P1+(bx/2)*u+h*v;
Ra=norm(P1-P0);
%% 求P0-RST坐标系到O-XYZ坐标系的变换关系T
r1=P1-P0;
t1=cross(r1,P2-P0);
r=r1/norm(r1);
t=t1/norm(t1);
s=cross(t,r);
R=[r',s',t'];
T=[   R   , P0';
   [0,0,0],  1 ];
iT=[  R'   ,-R'*P0';
    [0,0,0],   1  ];
%% 轨迹规划步骤
OP=[P1',P2',P3';  % 三个坐标点在O-XYZ坐标系的表示
    1  ,1  ,1  ]; 
Pp=iT*OP; % 三个坐标点在P0-RST坐标系的表示
Pp1=Pp(1:3,1)'; % P0-RST坐标系下的三坐标点
Pp2=Pp(1:3,2)';
Pp3=Pp(1:3,3)';

th12=atan2(Pp2(2),Pp2(1));
th13=atan2(Pp3(2),Pp3(1));
if th13<0
    th13=th13+2*pi;
end
th1=linspace(0,th12,N(1)+2);
th2=linspace(th12,th13,N(2)+2);
th=[th1,th2(2:end)];  %得到角度序列


px=Ra*cos(th);
py=Ra*sin(th);
pz(length(th))=0;
PP=[px;py;pz;pz+1]; %P系下的轨迹序列
P=T*PP;P(4,:)=[];P=P'; % 轨迹序列
end

