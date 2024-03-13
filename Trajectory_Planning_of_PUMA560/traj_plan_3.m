function [qq,vv,aa,tt] = traj_plan_3(q,v,t,N)
%[qq,vv,aa,tt] = traj_plan_3(q,v,t,N)   （m-1）段三次多项式轨迹规划
%   参数：起末点的位置，速度，时间（m维行向量）；
%         插值个数N(m-1维行向量)
%   结果：插值序列qq,vv,aa和时间序列tt
%
for i=1:length(q)-1 %每一段规划
    
T=t(i+1)-t(i); %时间间隔
ai0=q(i);%多项式系数
ai1=v(i);
ai2=(q(i+1)-q(i))*3/(T^2)-(2*v(i)+v(i+1))/T;
ai3=(q(i)-q(i+1))*2/(T^3)+(v(i)+v(i+1))/(T^2);

ti=linspace(t(i),t(i+1),N(i)+2);%单段t q v a序列的求解
qi=ai0+ai1*(ti-t(i))+ai2*(ti-t(i)).^2+ai3*(ti-t(i)).^3;
vi=ai1+2*ai2*(ti-t(i))+3*ai3*(ti-t(i)).^2;
ai=2*ai2+6*ai3*(ti-t(i));

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