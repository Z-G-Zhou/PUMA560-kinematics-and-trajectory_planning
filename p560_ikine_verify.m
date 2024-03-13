theta_plan=[10 -20 30 -10 20 -30];  
[T_plan,pos_plan]=p560_fkine(theta_plan); %随便指定一个关节角，生成一个pos_plan
[theta_solve_all,theta_solve_true]=p560_ikine(pos_plan);
theta_solve_all
theta_solve_true
T(4,4,8)=0;
for i=1:8
T(:,:,i)=p560_fkine(theta_solve_all(i,:));
end
T