function [qq,vv,aa,tt] = traj_plan_5(q,v,a,t,N)
%[qq,vv,aa,tt] = traj_plan_5(q,v,a,t,N) 五次多项式轨迹规划
%   参数：起末点的位置，速度，加速度，时间（二维行向量）；
%         插值个数N
%   结果：插值序列qq,vv,aa和时间序列tt
%
for i=1:length(q)-1 %每一段规划
    
T=t(i+1)-t(i); %时间间隔
ai0=q(i);  %多项式系数
ai1=v(i);
ai2=a(i)/2;
ai3=(20*q(i+1)-20*q(i)-(8*v(i+1)+12*v(i))*T-(3*a(i)-a(i+1))*T^2)/(2*T^3);
ai4=(30*q(i)-30*q(i+1)+(14*v(i+1)+16*v(i))*T+(3*a(i)-2*a(i+1))*T^2)/(2*T^4);
ai5=(12*q(i+1)-12*q(i)-(6*v(i+1)+6*v(i))*T-(a(i)-a(i+1))*T^2)/(2*T^5);

ti=linspace(t(i),t(i+1),N(i)+2);%单段t q v a序列的求解
qi=ai0+ai1*(ti-t(i))+ai2*(ti-t(i)).^2+ai3*(ti-t(i)).^3+ai4*(ti-t(i)).^4+ai5*(ti-t(i)).^5;
vi=ai1+2*ai2*(ti-t(i))+3*ai3*(ti-t(i)).^2+4*ai4*(ti-t(i)).^3+5*ai5*(ti-t(i)).^4;
ai=2*ai2+6*ai3*(ti-t(i))+12*ai4*(ti-t(i)).^2+20*ai5*(ti-t(i)).^3;

    if i==1
        tt=ti;
        qq=qi;
        vv=vi;
        aa=ai;
    else
        tt=[tt,ti(2:end)];
        qq=[qq,qi(2:end)];
        vv=[vv,vi(2:end)];
        aa=[aa,ai(2:end)];
    end
end
end