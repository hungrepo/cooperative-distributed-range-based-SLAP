function Main_Hybrid
% Initialization
 [tf,Ts,MPC,Mode,Noise,Tracker,Target,omega_bar]=TLAT_Initilization3D();
%% Start main Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
time=[];
Epursuit1=[];
Eest1=[];
for t = 0:Ts:tf
    time=[time; t];
    
    
    if t==0
       gamma1= 5.497787143782138;   
       gamma1_dot=.0;
       gamma_0_1=0;
       Rx1=80;
       Ry1=80;
       Rz1=50;
       
%        omega_bar=0.05;

%         Target.Id1.Pos=1*[5*sin(0.01*t); 0.1*t; -0.2*t ];
%         Target.Id1.Vel=1*[0.05*cos(0.01*t);  0.1  ; -0.2  ];

       Target.Id1.Pos=1*[30*sin(0.01*t); .1*t;   -0.5*t ];
       Target.Id1.Vel=1*[0.3*cos(0.01*t);  .1  ; -0.5  ];
       
       Target.Id1.State=[Target.Id1.Pos; Target.Id1.Vel];

       

       pd1=Target.Id1.Pos + [Rx1*cos(gamma1+gamma_0_1);
                             Ry1*sin(gamma1+gamma_0_1);
                             Rz1*sin(.5*gamma1+gamma_0_1)];

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
        pd1_hat(:,end+1)=Target.Id1.Est1(1:3,end) + [Rx1*cos(gamma1(end)+gamma_0_1);
                                                     Ry1*sin(gamma1(end)+gamma_0_1);
                                                     Rz1*sin(.5*gamma1(end)+gamma_0_1)];
        q_hat_dot1(:,end+1)=Target.Id1.Est1(4:6,end);

       
        Eest1= [Eest1; norm(Target.Id1.State(:,end)-Target.Id1.Est1(:,end))];                                        

        
        %delta=-0.5;
        epsilon=-1*[1; 1; 0];
        Delta=[1  0          -epsilon(3)  epsilon(2) ; 
               0  epsilon(3)  0          -epsilon(1) ;
               0 -epsilon(2)  epsilon(1)  0          ];
        Delta_bar=Delta'*inv(Delta*Delta');   
        K=1*diag([.5,.5, .5]);   
        kz=200;
      
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

 phi1=Tracker.Id1.State(4,end);
 theta1=Tracker.Id1.State(5,end);
 psi1=Tracker.Id1.State(6,end);

Tracker.Id1.Pos=Tracker.Id1.State(1:3,end);



Rot1 =  Rotation_matrix(phi1,theta1,psi1);       
    
         
   

%% True Target information
%% Estimated target information
r11=norm([Tracker.Id1.Pos-Target.Id1.Pos(:,end);Target.Id1.Depth])+0.5*randn;


% 

 
[Target.Id1.Est1(:,end+1),P1Out] = STST_EKF_PosVel_3D(r11,Tracker.Id1.Pos,Target.Id1.Est1(:,end),P1,Ts,t) ;
  P1=P1Out;
 
  
P1_save=[P1_save P1];
 
 %% With true target
 
% pd1_hat(:,end)= pd1(:,end);
%                                             
%  pd2_hat(:,end+1)= pd2(:,end);
%  
%  pd3_hat(:,end+1)= pd3(:,end);
% q_hat_dot1(:,end)=q_dot(:,end);
%  q_hat_dot2(:,end+1)=q_dot(:,end);
%  q_hat_dot3(:,end+1)=q_dot(:,end);

%  
%% Tracking error


%e=Rot1'*(Tracker.Id1.Pos(1:2,end)-pd_hat(:,end));
%% ----------------------------------- 
%% Coordination controller
% Network 1->2->3->1
uc1=0;
duc1=0;
%% Update gammas
r_gamma1=[      -Rx1*sin(gamma1(end)+gamma_0_1);
                 Ry1*cos(gamma1(end)+gamma_0_1);
             0.5*Rz1*cos(.5*gamma1(end)+gamma_0_1)];
 

e_gamma1=gamma1_dot(end)-omega_bar-uc1;
 

%% Controller

e1=Rot1'*(Tracker.Id1.Pos-pd1_hat(:,end))-epsilon; 
e1_true=Rot1'*(Tracker.Id1.Pos-pd1(:,end))-epsilon; 

 
Epursuit1=[Epursuit1;norm([e1_true;e_gamma1])]; 

%% ------------------------------------

% control parameter

% control law  
% upf1=Delta_inv*(Rot1'*(q_hat_dot(:,end)+r_gamma1*(omega_bar))-K*20*tanh(0.05*e1));
% upf2=Delta_inv*(Rot2'*(q_hat_dot(:,end)+r_gamma2*(omega_bar))-K*20*tanh(0.05*e2));
% upf3=Delta_inv*(Rot3'*(q_hat_dot(:,end)+r_gamma3*(omega_bar))-K*20*tanh(0.05*e3));

% 
upf1=Delta_bar*(Rot1'*(q_hat_dot1(:,end)+r_gamma1*(omega_bar+uc1))-K*e1);
 

ddgamma_ub=0.01;
ddgamma_lb=-0.001;
 gamma1_ddot=-kz*e_gamma1+e1'*Rot1'*r_gamma1 +duc1;
 gamma1_ddot=sat(gamma1_ddot,ddgamma_lb,ddgamma_ub);
 gamma1_dot(end+1)=gamma1_dot(end)+Ts*gamma1_ddot;
 gamma1(end+1)=gamma1(end)+Ts*gamma1_dot(end);
% gamma1(end+1)=gamma1(end)+Ts*.01;
 
% upf1=[0.5;0;0;0.01];

% upf1=inv(Delta)*(Rot1'*pd_hat_dot(:,end)-K*(e-[-delta; 0]));

Tracker.Id1.Input(:,end+1)=sat(upf1,Tracker.Id1.InputMin,Tracker.Id1.InputMax);
 
    
% Step 2: Update the vehicle position and orientation
Tracker.Id1.State(:,end+1)=update_vehicle(Tracker.Id1.State(:,end),Tracker.Id1.Input(:,end),Ts,t);
 

Target.Id1.Pos(:,end+1)=1*[30*sin(0.01*t); .1*t; -0.5*t ];
Target.Id1.Vel(:,end+1)=1*[0.3*cos(0.01*t);  .1  ; -0.5  ];
Target.Id1.State=[Target.Id1.Pos;Target.Id1.Vel];

%% Desire Trajectories
pd1(:,end+1)=Target.Id1.Pos(:,end)+ [Rx1*cos(gamma1(end)+gamma_0_1);
                                     Ry1*sin(gamma1(end)+gamma_0_1);
                                     Rz1*sin(.5*gamma1(end)+gamma_0_1)];
                                       
q_dot(:,end+1)=Target.Id1.Vel(:,end) ;



 %% With estimated target 
 pd1_hat(:,end+1)=Target.Id1.Est1(1:3,end) + [Rx1*cos(gamma1(end)+gamma_0_1);
                                              Ry1*sin(gamma1(end)+gamma_0_1);
                                              Rz1*sin(.5*gamma1(end)+gamma_0_1)];
%                                             
 q_hat_dot1(:,end+1)=Target.Id1.Est1(4:6,end);
 
 Eest1= [Eest1; norm(Target.Id1.State(:,end)-Target.Id1.Est1(:,end))];                                        

end
%% Save Data to Workspace
    Tracker.Id1.State=Tracker.Id1.State';


    
    Target.Id1.Pos=Target.Id1.Pos';
    
    pd1=pd1';
    pd1_hat=pd1_hat';

    
    p1=Tracker.Id1.State(:,1:3);

   
    Target_PosEst1=Target.Id1.Est1(1:3,:)';

    Target_Pos=Target.Id1.Pos;
    
    Target.Id1.Est1=Target.Id1.Est1';

    
    
    
    phi1=  Tracker.Id1.State(:,4)*180/pi;
    theta1=Tracker.Id1.State(:,5)*180/pi;
    psi1=  Tracker.Id1.State(:,6)*180/pi;

    
    save_to_base(1);
%% Run animation
%pd=x_path(:,1:2);
%pd_est=x_target_est(:,1:2);



% resample_1vehicles_3D
animation_1vehicles_3D
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
phi=x(4);
theta=x(5);
psi=x(6);
v=u(1);
p=u(2);
q=u(3);
r=u(4);
    
dx=[v*cos(psi)*cos(theta);
    v*sin(psi)*cos(theta);
   -v*sin(theta);
    p+q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta); 
    q*cos(phi)-r*sin(phi); 
    q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)];   
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
function Rot= Rotation_matrix(phi,theta,psi)
c_phi=cos(phi);
c_theta=cos(theta);
c_psi=cos(psi);
s_phi=sin(phi);
s_theta=sin(theta);
s_psi=sin(psi);
Rot =  [  c_psi*c_theta  -s_psi*c_phi+ c_psi*s_theta*s_phi   s_psi*s_phi+c_psi*c_phi*s_theta;
          s_psi*c_theta   c_psi*c_phi+ s_phi*s_theta*s_psi  -c_psi*s_phi+s_theta*s_psi*c_phi;
         -s_theta         c_theta*s_phi                      c_theta*c_phi];           
end