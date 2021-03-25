function Main_Hybrid
% Initialization
 [tf,Ts,MPC,Mode,Noise,Tracker,Target]=TLAT_Initilization_Distributed();
%% Start main Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for t = 0:Ts:tf
    if t==0
         gamma1= pi;
        gamma2= pi/2+pi/4;
        gamma3= 2*pi/2;
        gamma1_dot=.0;
        gamma2_dot=.0;
        gamma3_dot=.0;
        phi1=0;
        phi2=2*pi/3;
        phi3=4*pi/3;
        R1=20;
        R2=20;
        R3=20;
        omega=0.05;
        pd1=Target.Id1.Pos + [R1*cos(gamma1+phi1);
                              R1*sin(gamma1+phi1)];


        pd2=Target.Id1.Pos + [R2*cos(gamma2+phi2);
                              R2*sin(gamma2+phi2)];

                     
        pd3=Target.Id1.Pos + [R3*cos(gamma3+phi3);
                              R3*sin(gamma3+phi3)];

        q_dot=Target.Id1.Vel ;
        
        pd1_hat=[];
        pd2_hat=[];
        pd3_hat=[];
        q_hat_dot1=[];
        q_hat_dot2=[];
        q_hat_dot3=[];
       
%          pd_hat=Target.Id1.PosEst0 + [R*cos(omega*t);
%                                       R*sin(omega*t)];
%          pd_hat_dot=Target.Id1.EstVel0+[-R*omega*sin(omega*t);
%                                          R*omega*cos(omega*t)];              
                              
        P1=diag([250,200,.5,.5]);
        P2=diag([50,50,.5,.5]);
        P3=diag([50,50,.5,.5]);
        
        P1_save=P1;
        P2_save=P2;
        P3_save=P3;
        
        Target.Id1.Est1=[Target.Id1.PosHat0+[-20;-15]; Target.Id1.VelHat0];
        Target.Id1.Est2=[Target.Id1.PosHat0+[-0;-40]; Target.Id1.VelHat0];
        Target.Id1.Est3=[Target.Id1.PosHat0+[ 20; 10]; Target.Id1.VelHat0];
        
        
        delta=-0.5;
        epsilon=[delta; 0];
        Delta=[1 0; 
               0 -delta];
        Delta_inv=inv(Delta);   
        K=diag([.1,.05]);   
        kz=.10;
      
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
%     omega=omega+0.0001;
%     if omega > 0.2;
%         omega=0.2;
%     end
% end
%    
    
%% Tracker information

Tracker.Id1.Yaw=Tracker.Id1.State(3,end);
Tracker.Id1.Pos=Tracker.Id1.State(1:2,end);
Rot1 =   [cos(Tracker.Id1.Yaw) -sin(Tracker.Id1.Yaw);
          sin(Tracker.Id1.Yaw)  cos(Tracker.Id1.Yaw)];
      
Tracker.Id2.Yaw=Tracker.Id2.State(3,end);
Tracker.Id2.Pos=Tracker.Id2.State(1:2,end);
Rot2 =   [cos(Tracker.Id2.Yaw) -sin(Tracker.Id2.Yaw);
          sin(Tracker.Id2.Yaw)  cos(Tracker.Id2.Yaw)];      

      
Tracker.Id3.Yaw=Tracker.Id3.State(3,end);
Tracker.Id3.Pos=Tracker.Id3.State(1:2,end);
Rot3 =   [cos(Tracker.Id3.Yaw) -sin(Tracker.Id3.Yaw);
          sin(Tracker.Id3.Yaw)  cos(Tracker.Id3.Yaw)];            

%% True Target information
%% Estimated target information
r11=norm([Tracker.Id1.Pos-Target.Id1.Pos(:,end);Target.Id1.Depth])+0.5*randn;
r12=norm([Tracker.Id2.Pos-Target.Id1.Pos(:,end);Target.Id1.Depth])+0.5*randn;
r13=norm([Tracker.Id3.Pos-Target.Id1.Pos(:,end);Target.Id1.Depth])+0.5*randn;

% Compute local correction
 [Target.Id1.Est1(:,end+1),Target.Id1.Est2(:,end+1),Target.Id1.Est3(:,end+1),P1Out,P2Out,P3Out] = DEKF(r11,r12,r13,Tracker.Id1.Pos,Tracker.Id2.Pos,Tracker.Id3.Pos,Target.Id1.Est1(:,end),Target.Id1.Est2(:,end),Target.Id1.Est3(:,end),P1,P2,P3,Ts,Target.Id1.Depth,t);
  P1=P1Out;
  P2=P2Out;
  P3=P3Out;  
  
  
P1_save=[P1_save P1];
P2_save=[P2_save P2];
P3_save=[P3_save P3];  
  
  
 pd1_hat(:,end+1)=Target.Id1.Est1(1:2,end) + [R1*cos(gamma1(end)+phi1);
                                              R1*sin(gamma1(end)+phi1)];
                                            
 pd2_hat(:,end+1)=Target.Id1.Est2(1:2,end) + [R2*cos(gamma2(end)+phi2);
                                              R2*sin(gamma2(end)+phi2)];
 
 pd3_hat(:,end+1)=Target.Id1.Est3(1:2,end) + [R3*cos(gamma3(end)+phi3);
                                              R3*sin(gamma3(end)+phi3)];
 q_hat_dot1(:,end+1)=Target.Id1.Est1(3:4,end);
 q_hat_dot2(:,end+1)=Target.Id1.Est2(3:4,end);
 q_hat_dot3(:,end+1)=Target.Id1.Est3(3:4,end);

%% Tracking error


%e=Rot1'*(Tracker.Id1.Pos(1:2,end)-pd_hat(:,end));
%% ----------------------------------- 
%% Coordination controller
% Network 1->2->3->1

k2=.01;

uc1=-k2*(gamma1(end)-gamma3(end));
uc2=-k2*(gamma2(end)-gamma1(end));
uc3=-k2*(gamma3(end)-gamma2(end));
%% Update gammas
r_gamma1=R1*[-sin(gamma1(end)+phi1);cos(gamma1(end)+phi1)];
r_gamma2=R2*[-sin(gamma2(end)+phi2);cos(gamma2(end)+phi2)];
r_gamma3=R3*[-sin(gamma3(end)+phi3);cos(gamma3(end)+phi3)];

e_gamma1=gamma1_dot(end)-omega-uc1;
e_gamma2=gamma2_dot(end)-omega-uc2;
e_gamma3=gamma3_dot(end)-omega-uc3;

%% Controller

e1=Rot1'*(Tracker.Id1.Pos-pd1_hat(:,end))-epsilon; 
e2=Rot2'*(Tracker.Id2.Pos-pd2_hat(:,end))-epsilon;   
e3=Rot3'*(Tracker.Id3.Pos-pd3_hat(:,end))-epsilon;   

%% ------------------------------------

% control parameter

% control law  
% upf1=Delta_inv*(Rot1'*(q_hat_dot(:,end)+r_gamma1*(omega))-K*20*tanh(0.05*e1));
% upf2=Delta_inv*(Rot2'*(q_hat_dot(:,end)+r_gamma2*(omega))-K*20*tanh(0.05*e2));
% upf3=Delta_inv*(Rot3'*(q_hat_dot(:,end)+r_gamma3*(omega))-K*20*tanh(0.05*e3));

% 
upf1=Delta_inv*(Rot1'*(q_hat_dot1(:,end)+r_gamma1*(omega+uc1))-K*e1);
upf2=Delta_inv*(Rot2'*(q_hat_dot2(:,end)+r_gamma2*(omega+uc2))-K*e2);
upf3=Delta_inv*(Rot3'*(q_hat_dot3(:,end)+r_gamma3*(omega+uc3))-K*e3);


% gamma1_ddot=-kz*e_gamma1+e1'*Rot1'*r_gamma1+uc1;
% gamma1_ddot=sat(gamma1_ddot,-0.005,0.005);
% gamma1_dot(end+1)=sat(gamma1_dot(end)+Ts*gamma1_ddot,-inf,inf);
% gamma1(end+1)=gamma1(end)+Ts*gamma1_dot(end);
 gamma1(end+1)=gamma1(end)+Ts*(omega+uc1);


% gamma2_ddot=-kz*e_gamma2+e2'*Rot2'*r_gamma2+uc2;
% gamma2_ddot=sat(gamma2_ddot,-0.005,0.005);
% gamma2_dot(end+1)=sat(gamma2_dot(end)+Ts*gamma2_ddot,-inf,inf);
% gamma2(end+1)=gamma2(end)+Ts*gamma2_dot(end);
 gamma2(end+1)=gamma2(end)+Ts*(omega+uc2);

% gamma3_ddot=-kz*e_gamma3+e3'*Rot3'*r_gamma3+uc3;
% gamma3_ddot=sat(gamma3_ddot,-0.005,0.005);
% gamma3_dot(end+1)=sat(gamma3_dot(end)+Ts*gamma3_ddot,-inf,inf);
% gamma3(end+1)=gamma3(end)+Ts*gamma3_dot(end);
 gamma3(end+1)=gamma3(end)+Ts*(omega+uc3);


% upf1=inv(Delta)*(Rot1'*pd_hat_dot(:,end)-K*(e-[-delta; 0]));

Tracker.Id1.Input(:,end+1)=sat(upf1,Tracker.Id1.InputMin,Tracker.Id1.InputMax);
Tracker.Id2.Input(:,end+1)=sat(upf2,Tracker.Id1.InputMin,Tracker.Id1.InputMax);
Tracker.Id3.Input(:,end+1)=sat(upf3,Tracker.Id1.InputMin,Tracker.Id1.InputMax);
    
% Step 2: Update the vehicle position and orientation
Tracker.Id1.State(:,end+1)=update_vehicle(Tracker.Id1.State(:,end),Tracker.Id1.Input(:,end),Ts,t);
Tracker.Id2.State(:,end+1)=update_vehicle(Tracker.Id2.State(:,end),Tracker.Id2.Input(:,end),Ts,t);
Tracker.Id3.State(:,end+1)=update_vehicle(Tracker.Id3.State(:,end),Tracker.Id3.Input(:,end),Ts,t);



Target.Id1.Pos(:,end+1)=0*[20*sin(0.01*t +pi);  0.2*t ];
Target.Id1.Vel(:,end+1)=0*[0.2*cos(0.01*t+pi); 0.2    ];
%% Desire Trajectories
pd1(:,end+1)=Target.Id1.Pos(:,end)+ [R1*cos(gamma1(end)+phi1);
                                     R1*sin(gamma1(end)+phi1)];
                                       
pd2(:,end+1)=Target.Id1.Pos(:,end)+ [R2*cos(gamma2(end)+phi2);
                                     R2*sin(gamma2(end)+phi2)];


pd3(:,end+1)=Target.Id1.Pos(:,end)+ [R3*cos(gamma3(end)+phi3);
                                     R3*sin(gamma3(end)+phi3)];

q_dot(:,end+1)=Target.Id1.Vel(:,end) ;
                                 
end
%% Save Data to Workspace
    Tracker.Id1.State=Tracker.Id1.State';
    Tracker.Id2.State=Tracker.Id2.State';
    Tracker.Id3.State=Tracker.Id3.State';

    
    Target.Id1.Pos=Target.Id1.Pos';
    
    pd1=pd1';
    pd2=pd2';
    pd3=pd3';
    
    p1=Tracker.Id1.State(:,1:2);
    p2=Tracker.Id2.State(:,1:2);
    p3=Tracker.Id3.State(:,1:2);
   
    Target_PosEst1=Target.Id1.Est1(1:2,:)';
    Target_PosEst2=Target.Id1.Est2(1:2,:)';
    Target_PosEst3=Target.Id1.Est3(1:2,:)';
    Target_Pos=Target.Id1.Pos;
    
    
    
    Target.Id1.Est1=Target.Id1.Est1';
    Target.Id1.Est2=Target.Id1.Est2';
    Target.Id1.Est3=Target.Id1.Est3';
    
    
    
    yaw1=Tracker.Id1.State(:,3)*180/pi;
    yaw2=Tracker.Id2.State(:,3)*180/pi;
    yaw3=Tracker.Id3.State(:,3)*180/pi;
    save_to_base(1);
%% Run animation
%pd=x_path(:,1:2);
%pd_est=x_target_est(:,1:2);
% yaw1=Tracker.Id1.State(:,3)*180/pi;
% yaw2=Tracker.Id2.State(:,3)*180/pi;
% yaw3=Tracker.Id3.State(:,3)*180/pi;


resample_3vehicles
animation_3vehicles
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