function Test
p0=[5;20];
yaw0=pi;
x_robot=[p0;yaw0];
gamma=0;
gamma_dot=0;
x_gamma=[gamma; gamma_dot];
R=20;
pd=R*[cos(gamma);sin(gamma)];
pd_gamma=R*[-sin(gamma);cos(gamma)];
vd=0.05;
Ts=0.1;
delta=-0.5;
epsilon=[delta;0];
Delta_inv=[1  0;
           0 -1/delta];
kx=.5; ky=0.1;    
    Kk=[kx 0;
        0  ky];
kz=10;  
u_robot=[];
for t=1:Ts:200;
yaw=x_robot(3,end);
p=x_robot(1:2,end);
R_BI=[cos(yaw) -sin(yaw)
      sin(yaw)  cos(yaw)];
e=R_BI'*(p-pd(:,end))-epsilon;

u_pf= Delta_inv*(-Kk*20*tanh(.05*e)+R_BI'*pd_gamma(:,end)*vd);
u_robot(:,end+1)=sat(u_pf,[0;-0.4],[2;0.4]);
    
 %% control gamma   
 
% gamma_ddot=-kz*(gamma_dot(end)-vd)+e'*R_BI'*pd_gamma(:,end); 
% gamma_ddot=sat(gamma_ddot,-0.005,0.005);
% gamma_dot(end+1)=sat(gamma_dot(end)+Ts*gamma_ddot,0,inf);
% gamma(end+1)=gamma(end)+Ts*gamma_dot(end);

%% Update
x_robot(:,end+1)=update_vehicle(x_robot(:,end),u_robot(:,end),Ts,t);

% x_gamma(:,end+1)=update_gamma(x_gamma(:,end),gamma_ddot,Ts,t);
% gamma(end+1)=x_gamma(1,end);
% gamma_dot(end+1)=x_gamma(2,end);
gamma(end+1)=gamma(end)+Ts*vd;
pd(:,end+1)=R*[cos(gamma(end));sin(gamma(end))];
pd_gamma(:,end+1)=R*[-sin(gamma(end));cos(gamma(end))];
end
    save_to_base(1);

end

function x_next=update_vehicle(x_current,input,Ts,t)
    input_robot.nSteps = 4;
    input_robot.Ts=Ts;
    input_robot.u=input;
    input_robot.x = x_current;
    output_robot = RK4_integrator(@vehicle, input_robot);
    x_next = output_robot.value;
end
function x_next=update_gamma(x_current,input,Ts,t)
    input_gamma.nSteps = 4;
    input_gamma.Ts=Ts;
    input_gamma.u=input;
    input_gamma.x = x_current;
    output_gamma = RK4_integrator(@gamma_system, input_gamma);
    x_next = output_gamma.value;
end
%% vehicle dynamics
function dx = vehicle(t,x,u)
    psi=x(3);
    ur=u(1);
    r=u(2);
    dx=[ur*cos(psi);ur*sin(psi);r];
end
function dx = gamma_system(t,x,u)
    A=[0 1;
       0 0];
    B=[0;1];
    dx=A*x+B*u;
end
function y=sat(u,lowbound,upbound)
    y=max(min(u,upbound),lowbound);
end