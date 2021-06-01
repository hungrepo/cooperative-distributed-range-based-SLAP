%% This code is used to generate simulation results for 3 trackers - one target in the paper entitled: 

%   Cooperative distributed estimation and control of multiple autonomous vehicles 
%   for range-based underwater target localization and pursuit

%   Authors: Nguyen T. Hung, Francisco Rego, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/
         
%% =========================================================================================================================
function Main
% ------------------------------------Initialization-------------------------------------------------------
    tf = 200;                                                               % simulation time  
    Ts = 0.1;                                                               % sampling interval  
    N = 3;                                                                  % number of tracker used
    omega_bar = 0.1;
% Initialize the tracker's position and orietation
  %Tracker 1
    Tracker1.p = [55;-60; 25];                                                            % tracker position    
    Tracker1.eta = [pi;0;0];                                                             % tracker heading - yaw
    Tracker1.State = [Tracker1.p; Tracker1.eta];
    Tracker1.Input = [];                                                                 % linear and angular speeds
    
    Tracker1.InputMin = [ 0 ; -1; -1; -1];                                          % maximum linear and angular speeds
    Tracker1.InputMax = [10 ;  1;  1;  0.4];                                          % minimum linear and angular speeds
  % Tracker 2
    Tracker2.p = [80;35;-50];                                                            % tracker position    
    Tracker2.eta = [-pi/4;0;pi/2];                                                                % tracker heading - yaw
    Tracker2.State = [Tracker2.p; Tracker2.eta];
    Tracker2.Input = [];   
    
    Tracker2.InputMin = [ 0 ; -1; -1; -1];                                          % maximum linear and angular speeds
    Tracker2.InputMax = [10 ;  1;  1;  0.4];                                          % minimum linear and angular speeds
  % Tracker 3                                                                % linear and angular speeds   
    Tracker3.p = [-70; 75; -4];                                                            % tracker position    
    Tracker3.eta = [0; 0; 0];                                                                % tracker heading - yaw
    Tracker3.State = [Tracker3.p; Tracker3.eta];
    Tracker3.Input = [];                                                     % linear and angular speeds
    
    Tracker3.InputMin = [ 0 ; -1; -1; -1];                                          % maximum linear and angular speeds
    Tracker3.InputMax = [10 ;  1;  1;  0.4];                                          % minimum linear and angular speeds
  % Tracker input constraints      
% Initialize the target trajectory
    Target.Depth = 0;
    Target.q = [];                                                          % store target position
    Target.v = [];                                                          % store target velocity
    Target.x = [];                                                          % store target state
% The Spatial-Temporal (ST) curves for the vehicles to track       
    % Spatial parts (the circle about the target)     
      % for Tracker 1 
        Tracker1.gamma =  5.497787143782138;   Tracker1.gamma_dot = 0.0;
        Tracker1.rx = 80;   Tracker1.ry = 80;   Tracker1.rz = 50;   Tracker1.phi = 0;
      % for Tracker 2
        Tracker2.gamma = 4.712388980384690;  Tracker2.gamma_dot = 0.0;
        Tracker2.rx = 80;   Tracker2.ry = 80;   Tracker2.rz = 50;   Tracker2.phi = 2*pi/3;
      % for Tracker 3
        Tracker3.gamma = 4;   Tracker3.gamma_dot = 0.0;
        Tracker3.rx = 80;   Tracker3.ry = 80;   Tracker3.rz = 50;   Tracker3.phi = 4*pi/3;
% Initialize the estimate of target's trajectory       
  % At tracker 1  
    Tracker1.q_hat = [-30; 12; -25];                                                     % Initial estimate of target position
    Tracker1.v_hat = [0.2; 0.2; -0.1];                                                    % Initial estimate of target velocity 
    Tracker1.x_hat = [Tracker1.q_hat; Tracker1.v_hat];                                                 % Initial estimate of target state
    Tracker1.P_hat = diag([100,100,100,1,1,0.1]);                                       % Initial estimate of covariance
    Tracker1.P_save = Tracker1.P_hat;
  % At tracker 2
    Tracker2.q_hat = [20; -12; -21];                                                     % Initial estimate of target position
    Tracker2.v_hat = [0.2; 0.2; -0.1];                                                    % Initial estimate of target velocity 
    Tracker2.x_hat = [Tracker2.q_hat; Tracker2.v_hat];                                                 % Initial estimate of target state
    Tracker2.P_hat = diag([100,100,100,1,1,0.1]);                                         % Initial estimate of covariance
    Tracker2.P_save = Tracker2.P_hat;
  % At tracker 3  
    Tracker3.q_hat = [20; 20; -21];                                                     % Initial estimate of target position
    Tracker3.v_hat = [0.2; 0.2; -0.1];                                                    % Initial estimate of target velocity 
    Tracker3.x_hat = [Tracker3.q_hat; Tracker3.v_hat];                                                 % Initial estimate of target state
    Tracker3.P_hat = diag([200,100,100,1,1,0.1]);                                       % Initial estimate of covariance
    Tracker3.P_save = Tracker3.P_hat;      
% Set type of tracking controller
    TTC_type = 'TypeII';
     epsilon = -1*[1; 1; 0];
     Delta=[1  0          -epsilon(3)  epsilon(2) ; 
            0  epsilon(3)  0          -epsilon(1) ;
            0 -epsilon(2)  epsilon(1)  0          ];
 
    gains.epsilon = epsilon;
    gains.Delta_bar = Delta'*inv(Delta*Delta') ;   
    gains.K = 1*diag([.5,.5, .5]);   
    gains.kz = 200;         
    gains.gamma_ddot_max =  0.01;
    gains.gamma_ddot_min = -0.001;
% Variables to store information
    time = [];                                                              % Store time
    time_d = [];                                                            % Store descrete time
    Tracker1.E_Pursuit = [];    
    Tracker2.E_Pursuit = [];    
    Tracker3.E_Pursuit = [];                % Store pusuit errors
    Tracker1.E_Loc = [];        
    Tracker2.E_Loc = [];        
    Tracker3.E_Loc = [];                    % Store localization errors       
    Tracker1.range = [];        
    Tracker2.range = [];        
    Tracker3.range = [];                    % Store range measurement  
    Tracker1.pd = [];                                                                % S-T curve
    Tracker2.pd = [];                                                                % S-T curve
    Tracker3.pd = [];                                                                % S-T curve

    Tracker1.pd_dot = [];                                                            % derivative of pd
    Tracker2.pd_dot = [];                                                            % derivative of pd
    Tracker3.pd_dot = [];                                                            % derivative of pd
% Desired speed profile for gamma     
    Tracker1.omega = [];
    Tracker2.omega = [];
    Tracker3.omega = [];
% Correction speed in the coordination controller    
    Tracker1.vc = [];                                                       
    Tracker2.vc = [];
    Tracker3.vc = [];
% For target
    q = [];                                                                 % target position
    v = [];                                                                 % target velocity
    x = [];                                                                 % target state    
% gain of coordination controller    
    kc = 0.02;                                                          
% Store broadcast signals
  % Broadcast signals for transmiting gamma
    Tracker1.Com_Control = [];           
    Tracker2.Com_Control = [];
    Tracker3.Com_Control = [];
  % Broadcast signals for DEKF for transmiting the distribution
    Tracker1.Com_DEKF = [];
    Tracker2.Com_DEKF = [];
    Tracker3.Com_DEKF = [];
% Store variables for event triggered control
    Tracker1.gamma_hat = 0;
    Tracker2.gamma_hat = 0;
    Tracker3.gamma_hat = 0;
% Store variables for event triggered control
    Tracker1.gamma_tilde = [];
    Tracker2.gamma_tilde = [];
    Tracker3.gamma_tilde = [];
    
%------------------------------Start simulation ---------------------------

for t = 0:Ts:tf
  time(end+1) = t;   
  if rem(t,2)==0
    time_d(end+1) = t/2; 
  end
% compute correction speeds using the coordination controller 
    [Tracker1, Tracker2, Tracker3] = Coordination_CC(Tracker1, Tracker2, Tracker3, kc);     
    Tracker1.omega(end+1) = omega_bar + Tracker1.vc(end);
    Tracker2.omega(end+1) = omega_bar + Tracker2.vc(end);
    Tracker3.omega(end+1) = omega_bar + Tracker3.vc(end);
% Simulate/update position and velocity of target 
   % Note: if you want to change the desired target trajectory to track, change q and v
    Target.q(:,end+1) = [30*sin(0.01*t);   .1*t;   -0.5*t ];                             % target position
    Target.v(:,end+1) = [0.3*cos(0.01*t);  .1  ;   -0.5 ];                             % target velocity
    Target.x(:,end+1) = [Target.q(:,end); Target.v(:,end)];                                      % target state
% Update vehicles states
    Tracker1 = ST_Tracking_3D(Tracker1, Target, t, Ts, TTC_type, gains);
    Tracker2 = ST_Tracking_3D(Tracker2, Target, t, Ts, TTC_type, gains);
    Tracker3 = ST_Tracking_3D(Tracker3, Target, t, Ts, TTC_type, gains);

% Simulate ranges to target     
    Tracker1.range(end+1) = norm(Tracker1.p(:,end) - Target.q(:,end)) + 0.5*randn;
    Tracker2.range(end+1) = norm(Tracker2.p(:,end) - Target.q(:,end)) + 0.5*randn;
    Tracker3.range(end+1) = norm(Tracker3.p(:,end) - Target.q(:,end)) + 0.5*randn;

% Estimate the target state using DEKF
    [Tracker1, Tracker2, Tracker3] = DEKF_3V_3D(Tracker1, Tracker2, Tracker3, Target,Ts,t);     % go with periodic communications
    
% Update localization errors 
    Tracker1.E_Loc(end+1) = norm(Target.x(:,end) - Tracker1.x_hat(:,end));       % update localization error at Tracker 1                            
    Tracker2.E_Loc(end+1) = norm(Target.x(:,end) - Tracker2.x_hat(:,end));       % update localization error at Tracker 2                            
    Tracker3.E_Loc(end+1) = norm(Target.x(:,end) - Tracker3.x_hat(:,end));       % update localization error at Tracker 3                            
    
 % Update trackers state, position and yaw (psi)
    Tracker1.State(:,end+1) = update_vehicle(Tracker1.State(:,end),Tracker1.Input(:,end),Ts,t);  
    Tracker2.State(:,end+1) = update_vehicle(Tracker2.State(:,end),Tracker2.Input(:,end),Ts,t);  
    Tracker3.State(:,end+1) = update_vehicle(Tracker3.State(:,end),Tracker3.Input(:,end),Ts,t);  
    
    Tracker1.p = Tracker1.State(1:3,:);
    Tracker2.p = Tracker2.State(1:3,:);
    Tracker3.p = Tracker3.State(1:3,:);
    Tracker1.eta = Tracker1.State(4:6,:);
    Tracker2.eta = Tracker2.State(4:6,:);
    Tracker3.eta = Tracker3.State(4:6,:);

% Update gamma
    Tracker1.gamma(end+1) = Tracker1.gamma(end) + Ts*Tracker1.gamma_dot(end);   
    Tracker2.gamma(end+1) = Tracker2.gamma(end) + Ts*Tracker2.gamma_dot(end);   
    Tracker3.gamma(end+1) = Tracker3.gamma(end) + Ts*Tracker3.gamma_dot(end);  
end 
 
 
%% Save Data to Workspace
    Tracker1.pd = Tracker1.pd';
    Tracker2.pd = Tracker2.pd';
    Tracker3.pd = Tracker3.pd';
    
    Tracker1.State = Tracker1.State';
    Tracker2.State = Tracker2.State';
    Tracker3.State = Tracker3.State';

    Tracker1.p = Tracker1.p';
    Tracker2.p = Tracker2.p';
    Tracker3.p = Tracker3.p';
   
    Tracker1.q_hat = Tracker1.x_hat(1:3,:)';
    Tracker2.q_hat = Tracker2.x_hat(1:3,:)';
    Tracker3.q_hat = Tracker3.x_hat(1:3,:)';
    
    Target.q = Target.q';
    
    Tracker1.x_hat = Tracker1.x_hat';
    Tracker2.x_hat = Tracker2.x_hat';
    Tracker3.x_hat = Tracker3.x_hat';
    
    
%    Target.Id1.State = Target.x';

    
    P1_save = Tracker1.P_save;
    P2_save = Tracker2.P_save;
    P3_save = Tracker3.P_save;
    save_to_base(1);

animation_3vehicles_3D
end
%% ====================== Auxilary functions =============================
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
