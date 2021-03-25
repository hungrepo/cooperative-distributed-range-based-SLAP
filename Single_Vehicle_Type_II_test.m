function Main_Hybrid
% Initialization
 [tf,Ts,MPC,Mode,Noise,Tracker,Target,omega_bar]=TLAT_Initilization_2D_test();
%% Start main Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 0:Ts:tf
    if t==0
       gamma1= -pi/4;   
       gamma1_dot=.0;
       phi1=0;
       R1=20;
%        omega_bar=0.05;
        Target.Id1.Pos=0*[5*sin(0.01*t); 0.1*t ];
        Target.Id1.Vel=0*[0.05*cos(0.01*t);  0.1];
       
       pd1=Target.Id1.Pos +  [R1*cos(gamma1+phi1);
                              R1*sin(gamma1+phi1)];

        q_dot=Target.Id1.Vel ; 
        pd1_hat=[];
        q_hat_dot1=[];
        
       
%          pd_hat=Target.Id1.PosEst0 + [R*cos(omega_bar*t);
%                                       R*sin(omega_bar*t)];
%          pd_hat_dot=Target.Id1.EstVel0+[-R*omega_bar*sin(omega_bar*t);
%                                          R*omega_bar*cos(omega_bar*t)];              
                              
        P1=Target.Id1.P1;
   
        
        P1_save=P1;
        
        
        Target.Id1.Est1=[Target.Id1.PosHat1; Target.Id1.VelHat0];
        pd1_hat(:,end+1)=Target.Id1.Est1(1:2,end) + [R1*cos(gamma1(end)+phi1);
                                                     R1*sin(gamma1(end)+phi1)];
       
        
        
 
        epsilon=[1; 0];
        Delta=[1   epsilon(2)
               0  -epsilon(1)];
        Delta_bar=Delta'*inv(Delta*Delta');
        Delta_inv=Delta_bar;   
        K=diag([0.05,0.05]);   
        kz=.1;
      
    end
%% Change radius of and anglular speed of circle
% epsilon=0.01;
% if R>max(eig(P))
%     R=R-epsilon;
% else
%     R=R+epsilon;
% end
%    
% if R<6
%     R=6;
%     omega_bar=omega_bar+0.0001;
%     if omega_bar > 0.2;
%         omega_bar=0.2;
%     end
% end
%    
    
%% Tracker information

Tracker.Id1.Yaw=Tracker.Id1.State(3,end);
Tracker.Id1.Pos=Tracker.Id1.State(1:2,end);
Rot1 =   [cos(Tracker.Id1.Yaw) -sin(Tracker.Id1.Yaw);
          sin(Tracker.Id1.Yaw)  cos(Tracker.Id1.Yaw)];
      
   

%% True Target information
%% Estimated target information
r11=norm([Tracker.Id1.Pos-Target.Id1.Pos(:,end);Target.Id1.Depth])+0.5*randn;


% Compute local correction
 
 
  
  
 %% With estimated target 
 pd1_hat(:,end+1)=Target.Id1.Est1(1:2,end) + [R1*cos(gamma1(end)+phi1);
                                              R1*sin(gamma1(end)+phi1)];
                                            
 q_hat_dot1(:,end+1)=Target.Id1.Est1(3:4,end);
 
[Target.Id1.Est1(:,end+1),P1Out] = STST_EKF_PosVel(r11,Tracker.Id1.Pos,Target.Id1.Est1(:,end),P1,Ts,Target.Id1.Depth,t) ;
  P1=P1Out;
 
  
  P1_save=[P1_save P1];
 
 %% With true target
 
 pd1_hat(:,end)= pd1(:,end);
%                                             
%  pd2_hat(:,end+1)= pd2(:,end);
%  
%  pd3_hat(:,end+1)= pd3(:,end);
 q_hat_dot1(:,end)=q_dot(:,end);
%  q_hat_dot2(:,end+1)=q_dot(:,end);
%  q_hat_dot3(:,end+1)=q_dot(:,end);
%% 
 
%% Tracking error


%e=Rot1'*(Tracker.Id1.Pos(1:2,end)-pd_hat(:,end));
%% ----------------------------------- 
%% Coordination controller
% Network 1->2->3->1
uc1=0;
duc1=0;
%% Update gammas
r_gamma1=R1*[-sin(gamma1(end)+phi1);cos(gamma1(end)+phi1)];
 

e_gamma1=gamma1_dot(end)-omega_bar-uc1;
 

%% Controller

e1=Rot1'*(Tracker.Id1.Pos-pd1_hat(:,end))-epsilon; 
 

%% ------------------------------------

% control parameter

% control law  
% upf1=Delta_inv*(Rot1'*(q_hat_dot(:,end)+r_gamma1*(omega_bar))-K*20*tanh(0.05*e1));
% upf2=Delta_inv*(Rot2'*(q_hat_dot(:,end)+r_gamma2*(omega_bar))-K*20*tanh(0.05*e2));
% upf3=Delta_inv*(Rot3'*(q_hat_dot(:,end)+r_gamma3*(omega_bar))-K*20*tanh(0.05*e3));

% 
upf1=Delta_inv*(Rot1'*(q_hat_dot1(:,end)+r_gamma1*(omega_bar+uc1))-K*e1);
 


%  gamma1_ddot=-kz*e_gamma1+e1'*Rot1'*r_gamma1 +duc1;
% % gamma1_ddot=sat(gamma1_ddot,0,0.005);
%  gamma1_dot(end+1)=sat(gamma1_dot(end)+Ts*gamma1_ddot,0.01,0.1);
%  gamma1(end+1)=gamma1(end)+Ts*gamma1_dot(end);
gamma1(end+1)=gamma1(end)+Ts*omega_bar;
 


% upf1=inv(Delta)*(Rot1'*pd_hat_dot(:,end)-K*(e-[-delta; 0]));

Tracker.Id1.Input(:,end+1)=sat(upf1,Tracker.Id1.InputMin,Tracker.Id1.InputMax);
 
    
% Step 2: Update the vehicle position and orientation
Tracker.Id1.State(:,end+1)=update_vehicle(Tracker.Id1.State(:,end),Tracker.Id1.Input(:,end),Ts,t);
 


Target.Id1.Pos(:,end+1)=0*[20*sin(0.01*t +pi);  0.2*t ];
Target.Id1.Vel(:,end+1)=0*[0.2*cos(0.01*t+pi); 0.2    ];
%% Desire Trajectories
pd1(:,end+1)=Target.Id1.Pos(:,end)+ [R1*cos(gamma1(end)+phi1);
                                     R1*sin(gamma1(end)+phi1)];
                                       
q_dot(:,end+1)=Target.Id1.Vel(:,end) ;
                                 
end
%% Save Data to Workspace
    Tracker.Id1.State=Tracker.Id1.State';


    
    Target.Id1.Pos=Target.Id1.Pos';
    
    pd1=pd1';
    pd1_hat=pd1_hat';

    
    p1=Tracker.Id1.State(:,1:2);

   
    Target_PosEst1=Target.Id1.Est1(1:2,:)';

    Target_Pos=Target.Id1.Pos;
    
    Target.Id1.Est1=Target.Id1.Est1';

    
    
    
    yaw1=Tracker.Id1.State(:,3)*180/pi;

    
    save_to_base(1);
%% Run animation
%pd=x_path(:,1:2);
%pd_est=x_target_est(:,1:2);



resample_1vehicles
animation_1vehicles
end
%% End main loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Dynamics of Path and Vehicle %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x_next=update_vehicle(x_current,input,Ts,t)
    input_robot.nSteps = 4;
    input_robot.Ts=Ts;
    input_robot.u=input;
    input_robot.x = x_current;
    output_robot = RK4_integrator(@vehicle, input_robot);
    x_next = output_robot.value;
end
%% vehicle dynamics
function dx = vehicle(t,x,u)
    psi=x(3);
    ur=u(1);
    r=u(2);
    dx=[ur*cos(psi);ur*sin(psi);r];
end
%% path dynamics
function x_next=update_path(x_current,input,pathtype,pathpar,Ts,t)
    input_path.nSteps = 4;
    input_path.Ts=Ts;
    input_path.u=input;
    input_path.x = [x_current(3);x_current(8)];
    path_par=[x_current(6);x_current(7)];
    output_path = RK4_integrator(@(t,x,u)path(t,x,u,path_par), input_path);
    psid=output_path.value(1); gamma=output_path.value(2);
    x_next=paths(pathtype,pathpar,gamma,psid,t);
end
function dx = path(t,x,u,path_data)
    hg=path_data(1);
    cg=path_data(2); 
    dx=[hg*cg;1]*u;
end
function y=sat(u,lowbound,upbound)
    y=max(min(u,upbound),lowbound);
end
function x_next = update_target(x_current,input,Ts,t)
    input_target.nSteps = 4;
    input_target.Ts=Ts;
    input_target.u=input;
    input_target.x=x_current;
    output_target = RK4_integrator(@target, input_target);
    x_next = output_target.value;
end
function dx=target(t,x,u)
dx=u;
end
function dx=gamma_system(t,x,u)
A=[0 1;
   0 0];
B=[0;1];
dx=A*x+B*u;
end