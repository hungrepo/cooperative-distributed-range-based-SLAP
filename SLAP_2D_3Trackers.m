%% This code is used to generate simulation results for 3 trackers - one target in the paper entitled: 

%   Cooperative distributed estimation and control of multiple autonomous vehicles 
%   for range-based underwater target localization and pursuit

%   Authors: Nguyen T. Hung, Francisco Rego, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/
         
%% =========================================================================================================================
function Main
% ------------------------------------Initialization-------------------------------------------------------
    tf = 450;                                                               % simulation time  
    Ts = 0.1;                                                               % sampling interval  
    N = 3;                                                                  % number of tracker used
    omega_bar = 0.00;
% Initialize the tracker's position and orietation
  % Tracker 1
    Tracker1.p = [0;-50];                                                            % tracker position    
    Tracker1.psi = 0;                                                                % tracker heading - yaw
    Tracker1.State = [Tracker1.p; Tracker1.psi];
    Tracker1.Input = [];                                                     % linear and angular speeds
    
    Tracker1.InputMin = [0 ; -0.4];                                          % maximum linear and angular speeds
    Tracker1.InputMax = [2 ;  0.4];                                          % minimum linear and angular speeds
  % Tracker 2
    Tracker2.p = [30;20];                                                            % tracker position    
    Tracker2.psi = pi/2;                                                                % tracker heading - yaw
    Tracker2.State = [Tracker2.p; Tracker2.psi];
    Tracker2.Input = [];   
    
    Tracker2.InputMin = [0 ; -0.4];                                          % maximum linear and angular speeds
    Tracker2.InputMax = [2 ;  0.4];                                          % minimum linear and angular speeds
  % Tracker 3                                                                % linear and angular speeds   
    Tracker3.p = [0; 30];                                                            % tracker position    
    Tracker3.psi = -pi/2;                                                                % tracker heading - yaw
    Tracker3.State = [Tracker3.p; Tracker3.psi];
    Tracker3.Input = [];                                                     % linear and angular speeds
    
    Tracker3.InputMin = [0 ; -0.4];                                          % maximum linear and angular speeds
    Tracker3.InputMax = [2 ;  0.4];                                          % minimum linear and angular speeds
  % Tracker input constraints      
% Initialize the target trajectory
    Target.Depth = 0;
    Target.q = [];                                                          % store target position
    Target.v = [];                                                          % store target velocity
    Target.x = [];                                                          % store target state
% The Spatial-Temporal (ST) curves for the vehicles to track       
    % Spatial parts (the circle about the target)     
      % for Tracker 1 
        Tracker1.gamma =  2*pi-pi/4;   Tracker1.gamma_dot = 0.0;
        Tracker1.rx = 30;   Tracker1.ry = 30;   Tracker1.phi = 0;
      % for Tracker 2
        Tracker2.gamma = -pi/2+2*pi;  Tracker2.gamma_dot = 0.0;
        Tracker2.rx = 30;   Tracker2.ry = 30;   Tracker2.phi = 2*pi/3;
      % for Tracker 3
        Tracker3.gamma = 2*pi/2+pi/2-pi/8;   Tracker3.gamma_dot = 0.0;
        Tracker3.rx = 30;   Tracker3.ry = 30;   Tracker3.phi = 4*pi/3;
% Initialize the estimate of target's trajectory       
  % At tracker 1  
    Tracker1.q_hat = [-20; -15];                                                     % Initial estimate of target position
    Tracker1.v_hat = [-0.5; 0.5];                                                    % Initial estimate of target velocity 
    Tracker1.x_hat = [Tracker1.q_hat; Tracker1.v_hat];                                                 % Initial estimate of target state
    Tracker1.P_hat = diag([250,150, 0.5,0.5]);                                       % Initial estimate of covariance
    Tracker1.P_save = Tracker1.P_hat;
  % At tracker 2
    Tracker2.q_hat = [30; -35];                                                     % Initial estimate of target position
    Tracker2.v_hat = [-0.5; 0.5];                                                    % Initial estimate of target velocity 
    Tracker2.x_hat = [Tracker2.q_hat; Tracker2.v_hat];                                                 % Initial estimate of target state
    Tracker2.P_hat = diag([250,200, 0.5,0.5]);                                       % Initial estimate of covariance
    Tracker2.P_save = Tracker2.P_hat;
  % At tracker 3  
    Tracker3.q_hat = [30; 0];                                                     % Initial estimate of target position
    Tracker3.v_hat = [-0.5; 0.5];                                                    % Initial estimate of target velocity 
    Tracker3.x_hat = [Tracker3.q_hat; Tracker3.v_hat];                                                 % Initial estimate of target state
    Tracker3.P_hat = diag([150,150, 0.5,0.5]);                                       % Initial estimate of covariance
    Tracker3.P_save = Tracker3.P_hat;      
% Set type of tracking controller
    TTC_type = 'TypeII';
    delta = -1;
    Delta = [1 0; 
             0 -delta];
    gains.epsilon = [delta; 0];
    gains. Delta_inv = inv(Delta);   
    gains.K = diag([0.4, 0.2]);   
    gains.kz = 0.1;         
    gains.gamma_ddot_max =  0.01;
    gains.gamma_ddot_min = -0.001;
% Variables to store information
    time = [];                                                              % Store continous time
    time_d = [] ;                                                            % Store discrete time for EKF
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
% set communication mode
    ETC = false;                                                            
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
    Tracker1.gamma_hat = 3;
    Tracker2.gamma_hat = 3;
    Tracker3.gamma_hat = 3;
% Store variables for event triggered control
    Tracker1.gamma_tilde = [];
    Tracker2.gamma_tilde = [];
    Tracker3.gamma_tilde = [];
% Store triggering function for ETC coordination
    Tracker1.h = [];
    Tracker2.h = [];
    Tracker3.h = [];
% Store triggering functions for ETC DEKF
    Tracker1.g = [];
    Tracker2.g = [];
    Tracker3.g = [];
% Store Kullback-Leibler Divergence    
    Tracker1.KLD = [];
    Tracker2.KLD = [];
    Tracker3.KLD = [];
% Store reference propagation
    Tracker1.x_bar = Tracker1.x_hat; 
    Tracker2.x_bar = Tracker2.x_hat;
    Tracker3.x_bar = Tracker3.x_hat;
    
    Tracker1.P_bar = Tracker1.P_hat; 
    Tracker2.P_bar = Tracker2.P_hat;
    Tracker3.P_bar = Tracker3.P_hat;
    
%------------------------------Start simulation ---------------------------

for t = 0:Ts:tf
  time(end+1) = t;   
  if rem(t,2)==0
      time_d(end+1) = t/2; 
  end
% compute correction speeds using the coordination controller 

    if ETC 
        [Tracker1, Tracker2, Tracker3] = Coordination_ETC(Tracker1, Tracker2, Tracker3, kc, t);
    else 
        [Tracker1, Tracker2, Tracker3] = Coordination_CC(Tracker1, Tracker2, Tracker3, kc);     
    end
    
    Tracker1.omega(end+1) = omega_bar + Tracker1.vc(end);
    Tracker2.omega(end+1) = omega_bar + Tracker2.vc(end);
    Tracker3.omega(end+1) = omega_bar + Tracker3.vc(end);
% Simulate/update position and velocity of target 
   % Note: if you want to change the desired target trajectory to track, change q and v
    Target.q(:,end+1) = [20*sin(0.01*t +pi);  0.3*t ];                             % target position
    Target.v(:,end+1) = [0.2*cos(0.01*t+pi);  0.3   ];                             % target velocity
    Target.x(:,end+1) = [Target.q(:,end); Target.v(:,end)];                                      % target state
% Update vehicles states
    Tracker1 = ST_Tracking_2D(Tracker1, Target, t, Ts, TTC_type, gains);
    Tracker2 = ST_Tracking_2D(Tracker2, Target, t, Ts, TTC_type, gains);
    Tracker3 = ST_Tracking_2D(Tracker3, Target, t, Ts, TTC_type, gains);

% Simulate ranges to target     
    Tracker1.range(end+1) = norm([Tracker1.p(:,end) - Target.q(:,end);Target.Depth]) + 0.5*randn;
    Tracker2.range(end+1) = norm([Tracker2.p(:,end) - Target.q(:,end);Target.Depth]) + 0.5*randn;
    Tracker3.range(end+1) = norm([Tracker3.p(:,end) - Target.q(:,end);Target.Depth]) + 0.5*randn;

% Estimate the target state using DEKF
    
    if ETC 
        [Tracker1, Tracker2, Tracker3] = DEKF_3V_2D_ETC(Tracker1, Tracker2, Tracker3, Target,Ts,t);
    else 
        [Tracker1, Tracker2, Tracker3] = DEKF_3V_2D(Tracker1, Tracker2, Tracker3, Target,Ts,t);     % go with periodic communications
    end
    
% Update localization errors 
    Tracker1.E_Loc(end+1) = norm(Target.x(:,end) - Tracker1.x_hat(:,end));       % update localization error at Tracker 1                            
    Tracker2.E_Loc(end+1) = norm(Target.x(:,end) - Tracker2.x_hat(:,end));       % update localization error at Tracker 2                            
    Tracker3.E_Loc(end+1) = norm(Target.x(:,end) - Tracker3.x_hat(:,end));       % update localization error at Tracker 3                            
    
 % Update trackers state, position and yaw (psi)
    Tracker1.State(:,end+1) = update_vehicle(Tracker1.State(:,end),Tracker1.Input(:,end),Ts,t);  
    Tracker2.State(:,end+1) = update_vehicle(Tracker2.State(:,end),Tracker2.Input(:,end),Ts,t);  
    Tracker3.State(:,end+1) = update_vehicle(Tracker3.State(:,end),Tracker3.Input(:,end),Ts,t);  
    
    Tracker1.p = Tracker1.State(1:2,:);
    Tracker2.p = Tracker2.State(1:2,:);
    Tracker3.p = Tracker3.State(1:2,:);
    Tracker1.psi = Tracker1.State(3,:);
    Tracker2.psi = Tracker2.State(3,:);
    Tracker3.psi = Tracker3.State(3,:);

% Update gamma
    Tracker1.gamma(end+1) = Tracker1.gamma(end) + Ts*Tracker1.gamma_dot(end);   
    Tracker2.gamma(end+1) = Tracker2.gamma(end) + Ts*Tracker2.gamma_dot(end);   
    Tracker3.gamma(end+1) = Tracker3.gamma(end) + Ts*Tracker3.gamma_dot(end);  
    
    if ETC
        Tracker1.gamma_hat(end+1) = Tracker1.gamma_hat(end) + Ts*omega_bar;   
        Tracker2.gamma_hat(end+1) = Tracker2.gamma_hat(end) + Ts*omega_bar;
        Tracker3.gamma_hat(end+1) = Tracker3.gamma_hat(end) + Ts*omega_bar;
    end
end 
 
 
%% Save Data to Workspace
    Tracker1.pd = Tracker1.pd';
    Tracker2.pd = Tracker2.pd';
    Tracker3.pd = Tracker3.pd';

    Tracker1.p = Tracker1.p';
    Tracker2.p = Tracker2.p';
    Tracker3.p = Tracker3.p';
   
    Tracker1.q_hat = Tracker1.x_hat(1:2,:)';
    Tracker2.q_hat = Tracker2.x_hat(1:2,:)';
    Tracker3.q_hat = Tracker3.x_hat(1:2,:)';
    
    Target.q = Target.q';
    
    Tracker1.x_hat = Tracker1.x_hat';
    Tracker2.x_hat = Tracker2.x_hat';
    Tracker3.x_hat = Tracker3.x_hat';
    
    
%    Target.Id1.State = Target.x';
    yaw1 = Tracker1.psi*180/pi;
    yaw2 = Tracker2.psi*180/pi;
    yaw3 = Tracker3.psi*180/pi;
    
    P1_save = Tracker1.P_save;
    P2_save = Tracker2.P_save;
    P3_save = Tracker3.P_save;
    save_to_base(1);

animation_3vehicles
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
    psi=x(3);
    ur=u(1);
    r=u(2);
    dx=[ur*cos(psi);ur*sin(psi);r];
end
