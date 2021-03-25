function Main_Hybrid
% Initialization
 [tf,Ts,MPC,Mode,Noise,Tracker,Target,omega_bar]=TLAT_Initilization3D();
%% Start main Loop %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
N=1;
time=[];
Epursuit1=[];
Epursuit2=[];
Epursuit3=[];

Eest1=[];
Eest2=[];
Eest3=[];

for t = 0:Ts:tf
    time=[time; t];
    if t==0
        gamma1= pi/4;
        gamma2= -pi/2+2*pi;
        gamma3= 2*pi/2+pi/2;
        
        gamma1=5.497787143782138;
        gamma2=4.712388980384690;
        gamma3=4;
        gamma1_dot=.0;
        gamma2_dot=.0;
        gamma3_dot=.0;
        gamma_0_1=0;
        gamma_0_2=2*pi/3;
        gamma_0_3=4*pi/3;
        Rx1=80;
        Ry1=80;
        Rz1=50;
        
        Rx2=80;
        Ry2=80;
        Rz2=50;
        
        Rx3=80;
        Ry3=80;
        Rz3=50;
%        omega_bar=0.05;

       
           Target.Id1.Pos=1*[30*sin(0.01*t); .1*t;   -0.5*t ];
           Target.Id1.Vel=1*[0.3*cos(0.01*t);  .1  ; -0.5 ];
        
            Target.Id1.State=[Target.Id1.Pos; Target.Id1.Vel];

          pd1=Target.Id1.Pos + [Rx1*cos(gamma1+gamma_0_1);
                                Ry1*sin(gamma1+gamma_0_1);
                                Rz1*sin(.5*gamma1+gamma_0_1)];


          pd2=Target.Id1.Pos + [Rx2*cos(gamma2+gamma_0_2);
                                Ry2*sin(gamma2+gamma_0_2);
                                Rz2*sin(.5*gamma2+gamma_0_2)];

                     
          pd3=Target.Id1.Pos + [  Rx3*cos(gamma2+gamma_0_3);
                                  Ry3*sin(gamma3+gamma_0_3);
                                  Rz3*sin(.5*gamma3+gamma_0_3)];

        q_dot=Target.Id1.Vel ;
        
        pd1_hat=[];
        pd2_hat=[];
        pd3_hat=[];
        q_hat_dot1=[];
        q_hat_dot2=[];
        q_hat_dot3=[];
       
%          pd_hat=Target.Id1.PosEst0 + [R*cos(omega_bar*t);
%                                       R*sin(omega_bar*t)];
%          pd_hat_dot=Target.Id1.EstVel0+[-R*omega_bar*sin(omega_bar*t);
%                                          R*omega_bar*cos(omega_bar*t)];              
                              
 
        P1=Target.Id1.P1;
        P2=Target.Id1.P2;
        P3=Target.Id1.P3;
        P1_save=P1;
        P2_save=P2;
        P3_save=P3;
        
        Target.Id1.Est1=[Target.Id1.PosHat1; Target.Id1.VelHat0];
        Target.Id1.Est2=[Target.Id1.PosHat2; Target.Id1.VelHat0];
        Target.Id1.Est3=[Target.Id1.PosHat3; Target.Id1.VelHat0];
        
        
        pd1_hat(:,end+1)=Target.Id1.Est1(1:3,end) + [   Rx1*cos(gamma1+gamma_0_1);
                                                        Ry1*sin(gamma1+gamma_0_1);
                                                        Rz1*sin(.5*gamma1+gamma_0_1)];
        pd2_hat(:,end+1)=Target.Id1.Est2(1:3,end) + [   Rx2*cos(gamma2+gamma_0_2);
                                                        Ry2*sin(gamma2+gamma_0_2);
                                                        Rz2*sin(.5*gamma2+gamma_0_2)];
        pd3_hat(:,end+1)=Target.Id1.Est3(1:3,end) + [   Rx3*cos(gamma3+gamma_0_3);
                                                        Ry3*sin(gamma3+gamma_0_3);
                                                        Rz3*sin(.5*gamma3+gamma_0_3)];                                    
        
        q_hat_dot1(:,end+1)=Target.Id1.Est1(4:6,end);
        q_hat_dot2(:,end+1)=Target.Id1.Est2(4:6,end);
        q_hat_dot3(:,end+1)=Target.Id1.Est3(4:6,end);
                                            
                                                 
        Eest1= [Eest1; norm(Target.Id1.State(:,end)-Target.Id1.Est1(:,end))];                                        
        Eest2= [Eest2; norm(Target.Id1.State(:,end)-Target.Id1.Est2(:,end))];                                        
        Eest3= [Eest3; norm(Target.Id1.State(:,end)-Target.Id1.Est3(:,end))];                                        
                                        
        
        
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

phi2=Tracker.Id2.State(4,end);
theta2=Tracker.Id2.State(5,end);
psi2=Tracker.Id2.State(6,end);
Tracker.Id2.Pos=Tracker.Id2.State(1:3,end);

phi3=Tracker.Id3.State(4,end);
theta3=Tracker.Id3.State(5,end);
psi3=Tracker.Id3.State(6,end);
Tracker.Id3.Pos=Tracker.Id3.State(1:3,end);

Rot1 =  Rotation_matrix(phi1,theta1,psi1);       
Rot2 =  Rotation_matrix(phi2,theta2,psi2);       
Rot3 =  Rotation_matrix(phi3,theta3,psi3);       

      
         

%% True Target information
%% Estimated target information
r11=norm(Tracker.Id1.Pos-Target.Id1.Pos(:,end))+0.5*randn;
r12=norm(Tracker.Id2.Pos-Target.Id1.Pos(:,end))+0.5*randn;
r13=norm(Tracker.Id3.Pos-Target.Id1.Pos(:,end))+0.5*randn;

% Compute local correction
 [Target.Id1.Est1(:,end+1),Target.Id1.Est2(:,end+1),Target.Id1.Est3(:,end+1),P1Out,P2Out,P3Out] = DEKF_3V_3D(r11,r12,r13,Tracker.Id1.Pos,Tracker.Id2.Pos,Tracker.Id3.Pos,Target.Id1.Est1(:,end),Target.Id1.Est2(:,end),Target.Id1.Est3(:,end),P1,P2,P3,Ts,t);
  P1=P1Out;
  P2=P2Out;
  P3=P3Out;  
  
  P1_save=[P1_save P1];
  P2_save=[P2_save P2];
  P3_save=[P3_save P3];
  
  
 %% With estimated target 


 
 %% With true target
 
%  pd1_hat(:,end+1)= pd1(:,end);
%                                             
%  pd2_hat(:,end+1)= pd2(:,end);
%  
%  pd3_hat(:,end+1)= pd3(:,end);
%  q_hat_dot1(:,end+1)=q_dot(:,end);
%  q_hat_dot2(:,end+1)=q_dot(:,end);
%  q_hat_dot3(:,end+1)=q_dot(:,end);
%% 
 
%% Tracking error


%e=Rot1'*(Tracker.Id1.Pos(1:2,end)-pd_hat(:,end));
%% ----------------------------------- 
%% Coordination controller
% Network 1->2->3->1

k2=.02;

uc1=-k2*(gamma1(end)-gamma3(end));
uc2=-k2*(gamma2(end)-gamma1(end));
uc3=-k2*(gamma3(end)-gamma2(end));

duc1=-k2*(gamma1_dot(end)-gamma3_dot(end));
duc2=-k2*(gamma2_dot(end)-gamma1_dot(end));
duc3=-k2*(gamma3_dot(end)-gamma2_dot(end));
%% Update gammas
r_gamma1=[      -Rx1*sin(gamma1(end)+gamma_0_1);
                 Ry1*cos(gamma1(end)+gamma_0_1);
             0.5*Rz1*cos(.5*gamma1(end)+gamma_0_1)];
r_gamma2=[      -Rx2*sin(gamma2(end)+gamma_0_2);
                 Ry2*cos(gamma2(end)+gamma_0_2);
             0.5*Rz2*cos(.5*gamma2(end)+gamma_0_2)];
r_gamma3=[      -Rx3*sin(gamma3(end)+gamma_0_3);
                 Ry3*cos(gamma3(end)+gamma_0_3);
             0.5*Rz3*cos(.5*gamma3(end)+gamma_0_3)];         
         
e_gamma1=gamma1_dot(end)-omega_bar-uc1;
e_gamma2=gamma2_dot(end)-omega_bar-uc2;
e_gamma3=gamma3_dot(end)-omega_bar-uc3;

%% Controller

e1=Rot1'*(Tracker.Id1.Pos-pd1_hat(:,end))-epsilon; 
e2=Rot2'*(Tracker.Id2.Pos-pd2_hat(:,end))-epsilon;   
e3=Rot3'*(Tracker.Id3.Pos-pd3_hat(:,end))-epsilon;   


e1_true=Rot1'*(Tracker.Id1.Pos-pd1(:,end))-epsilon; 
e2_true=Rot2'*(Tracker.Id2.Pos-pd2(:,end))-epsilon;   
e3_true=Rot3'*(Tracker.Id3.Pos-pd3(:,end))-epsilon;   

Epursuit1=[Epursuit1;norm([e1_true;e_gamma1])]; 
Epursuit2=[Epursuit2;norm([e2_true;e_gamma2])]; 
Epursuit3=[Epursuit3;norm([e3_true;e_gamma3])]; 


%% ------------------------------------
ddgamma_ub=0.01;
ddgamma_lb=-0.001;
% control parameter

% control law  
% upf1=Delta_inv*(Rot1'*(q_hat_dot(:,end)+r_gamma1*(omega_bar))-K*20*tanh(0.05*e1));
% upf2=Delta_inv*(Rot2'*(q_hat_dot(:,end)+r_gamma2*(omega_bar))-K*20*tanh(0.05*e2));
% upf3=Delta_inv*(Rot3'*(q_hat_dot(:,end)+r_gamma3*(omega_bar))-K*20*tanh(0.05*e3));

% 
upf1=Delta_bar*(Rot1'*(q_hat_dot1(:,end)+r_gamma1*(omega_bar+uc1))-K*e1);
upf2=Delta_bar*(Rot2'*(q_hat_dot2(:,end)+r_gamma2*(omega_bar+uc2))-K*e2);
upf3=Delta_bar*(Rot3'*(q_hat_dot3(:,end)+r_gamma3*(omega_bar+uc3))-K*e3);


 gamma1_ddot=-kz*e_gamma1+e1'*Rot1'*r_gamma1 +duc1;
gamma1_ddot=sat(gamma1_ddot,ddgamma_lb,ddgamma_ub);
 gamma1_dot(end+1)=gamma1_dot(end)+Ts*gamma1_ddot;
 gamma1(end+1)=gamma1(end)+Ts*gamma1_dot(end);
% gamma1(end+1)=gamma1(end)+Ts*(omega_bar+uc1);


 gamma2_ddot=-kz*e_gamma2+e2'*Rot2'*r_gamma2+duc2;
 gamma2_ddot=sat(gamma2_ddot,ddgamma_lb,ddgamma_ub);
 gamma2_dot(end+1)=gamma2_dot(end)+Ts*gamma2_ddot;
 gamma2(end+1)=gamma2(end)+Ts*gamma2_dot(end);
% gamma2(end+1)=gamma2(end)+Ts*(omega_bar+uc2);

 gamma3_ddot=-kz*e_gamma3+e3'*Rot3'*r_gamma3+duc3;
 gamma3_ddot=sat(gamma3_ddot,ddgamma_lb,ddgamma_ub);
 gamma3_dot(end+1)=gamma3_dot(end)+Ts*gamma3_ddot;
 gamma3(end+1)=gamma3(end)+Ts*gamma3_dot(end);
% gamma3(end+1)=gamma3(end)+Ts*(omega_bar+uc3);


% upf1=inv(Delta)*(Rot1'*pd_hat_dot(:,end)-K*(e-[-delta; 0]));

Tracker.Id1.Input(:,end+1)=sat(upf1,Tracker.Id1.InputMin,Tracker.Id1.InputMax);
Tracker.Id2.Input(:,end+1)=sat(upf2,Tracker.Id1.InputMin,Tracker.Id1.InputMax);
Tracker.Id3.Input(:,end+1)=sat(upf3,Tracker.Id1.InputMin,Tracker.Id1.InputMax);
    
% Step 2: Update the vehicle position and orientation
Tracker.Id1.State(:,end+1)=update_vehicle(Tracker.Id1.State(:,end),Tracker.Id1.Input(:,end),Ts,t);
Tracker.Id2.State(:,end+1)=update_vehicle(Tracker.Id2.State(:,end),Tracker.Id2.Input(:,end),Ts,t);
Tracker.Id3.State(:,end+1)=update_vehicle(Tracker.Id3.State(:,end),Tracker.Id3.Input(:,end),Ts,t);


Target.Id1.Pos(:,end+1)=1*[30*sin(0.01*t); .1*t;   -0.5*t ];
Target.Id1.Vel(:,end+1)=1*[0.3*cos(0.01*t);  .1  ; -0.5 ];
Target.Id1.State=[Target.Id1.Pos;Target.Id1.Vel];

%% Desire Trajectories
pd1(:,end+1)=Target.Id1.Pos(:,end)+ [   Rx1*cos(gamma1(end)+gamma_0_1);
                                        Ry1*sin(gamma1(end)+gamma_0_1);
                                        Rz1*sin(.5*gamma1(end)+gamma_0_1)];
                                       
pd2(:,end+1)=Target.Id1.Pos(:,end)+ [   Rx2*cos(gamma2(end)+gamma_0_2);
                                        Ry2*sin(gamma2(end)+gamma_0_2);
                                        Rz2*sin(.5*gamma2(end)+gamma_0_2)];


pd3(:,end+1)=Target.Id1.Pos(:,end)+ [   Rx3*cos(gamma3(end)+gamma_0_3);
                                        Ry3*sin(gamma3(end)+gamma_0_3);
                                        Rz3*sin(.5*gamma3(end)+gamma_0_3)];


q_dot(:,end+1)=Target.Id1.Vel(:,end) ;


%% Update estimated Hybrid-PT
pd1_hat(:,end+1)=Target.Id1.Est1(1:3,end) + [  Rx1*cos(gamma1(end)+gamma_0_1);
                                               Ry1*sin(gamma1(end)+gamma_0_1);
                                               Rz1*sin(.5*gamma1(end)+gamma_0_1)];
                                            
 pd2_hat(:,end+1)=Target.Id1.Est2(1:3,end) + [   Rx2*cos(gamma2(end)+gamma_0_2);
                                                 Ry2*sin(gamma2(end)+gamma_0_2);
                                                 Rz2*sin(.5*gamma2(end)+gamma_0_2)];
 
 pd3_hat(:,end+1)=Target.Id1.Est3(1:3,end) + [   Rx3*cos(gamma3(end)+gamma_0_3);
                                                 Ry3*sin(gamma3(end)+gamma_0_3);
                                                 Rz3*sin(.5*gamma3(end)+gamma_0_3)];       
q_hat_dot1(:,end+1)=Target.Id1.Est1(4:6,end);
q_hat_dot2(:,end+1)=Target.Id1.Est2(4:6,end);
q_hat_dot3(:,end+1)=Target.Id1.Est3(4:6,end);


Eest1= [Eest1; norm(Target.Id1.State(:,end)-Target.Id1.Est1(:,end))];                                        
Eest2= [Eest2; norm(Target.Id1.State(:,end)-Target.Id1.Est2(:,end))];                                        
Eest3= [Eest3; norm(Target.Id1.State(:,end)-Target.Id1.Est3(:,end))];                                        

                                 
end
%% Save Data to Workspace
    Tracker.Id1.State=Tracker.Id1.State';
    Tracker.Id2.State=Tracker.Id2.State';
    Tracker.Id3.State=Tracker.Id3.State';

    Target.Id1.State=[Target.Id1.Pos;
                      Target.Id1.Vel];
    Target.Id1.Pos=Target.Id1.Pos';
    
    pd1=pd1';
    pd2=pd2';
    pd3=pd3';
    
    pd1_hat=pd1_hat';
    pd2_hat=pd2_hat';
    pd3_hat=pd3_hat';
    
    p1=Tracker.Id1.State(:,1:3);
    p2=Tracker.Id2.State(:,1:3);
    p3=Tracker.Id3.State(:,1:3);
   
    Target_PosEst1=Target.Id1.Est1(1:3,:)';
    Target_PosEst2=Target.Id1.Est2(1:3,:)';
    Target_PosEst3=Target.Id1.Est3(1:3,:)';
    Target_Pos=Target.Id1.Pos;
    
    Target.Id1.Est1=Target.Id1.Est1';
    Target.Id1.Est2=Target.Id1.Est2';
    Target.Id1.Est3=Target.Id1.Est3';
    
    
    Target.Id1.State=Target.Id1.State';
     
    save_to_base(1);
%% Run animation
%pd=x_path(:,1:2);
%pd_est=x_target_est(:,1:2);



% resample_3vehicles
animation_3vehicles_3D
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