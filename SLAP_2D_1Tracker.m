
%% This code is used to generate simulation results for one tracker- one target in the paper entitled: 

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
    N = 1;                                                                  % number of tracker used
    omega_bar = 0.05;                                                       % desired angular rate encircling about target  
% Initialize the tracker's position and orietation
    p = [0;-50];                                                            % tracker position    
    psi = 0;                                                                % tracker heading - yaw
    Tracker.State = [p; psi];
    Tracker.Input = [];                                                     % linear and angular speeds
    Tracker.InputMin = [0 ; -0.4];                                          % maximum linear and angular speeds
    Tracker.InputMax = [2 ;  0.4];                                          % minimum linear and angular speeds
% Initialize the target trajectory
    Target.Depth = 0;
% The Spatial-Temporal (ST) cure for the vehicle to track       
    % Spatial part     
        gamma = -pi/4;   
        gamma_dot = 0.0;
        phi = 0;
        rx = 30;
        ry = 30;           
% Initialize the estimate of target's trajectory       
    q_hat = [-20; -15];                                                     % Initial estimate of target position
    v_hat = [-0.5; 0.5];                                                    % Initial estimate of target velocity 
    x_hat = [q_hat; v_hat];                                                 % Initial estimate of target state
    P_hat = diag([250,150, 0.5,0.5]);                                       % Initial estimate of covariance
    P_save = P_hat;
% Set type of tracking controller
    TTC_type = 'TypeII';
% Set gains of the tracking controller 
    delta = -1;
    Delta = [1 0; 
             0 -delta];
    gains.epsilon = [delta; 0];
    gains. Delta_inv = inv(Delta);   
    gains.K = diag([0.4, 0.2]);   
    gains.kz = 500;         
    gains.gamma_ddot_max =  0.01;
    gains.gamma_ddot_min = -0.001;
% Variables to store information
    time = [];                                                              % Store time
    E_Pursuit = [];                                                         % Store pusuit error
    E_Loc = [];                                                             % Store localization error       
    range = [];                                                             % Store range measurement  
    pd = [];                                                                % S-T curve
    pd_dot = [];                                                            % derivative of pd
    q = [];                                                                 % target position
    v = [];                                                                 % target velocity
    x = [];                                                                 % target state
%------------------------------Start simulation ---------------------------
for t = 0:Ts:tf
    time(end+1) = t;   
    omega = omega_bar;
% Simulate/update position and velocity of target 
   % Note: if you want to change the desired target trajectory to track, change q and v
    q(:,end+1) = [20*sin(0.01*t +pi);  0.3*t ];                             % target position
    v(:,end+1) = [0.2*cos(0.01*t+pi);  0.3   ];                             % target velocity
    x(:,end+1) = [q(:,end); v(:,end)];                                      % target state

% Update desired S-T curve
    r_gamma = [rx*cos(gamma(end) + phi);                                      % spatial path (circle about target)
               ry*sin(gamma(end) + phi)]; 
    dr_gamma = [-rx*sin(gamma(end) + phi);                                    % partial derivative of r respect to gamma
                 ry*cos(gamma(end) + phi)];  
    pd(:,end+1) = q(:,end) + r_gamma;                                       % update S-T curve
    pd_dot(:,end+1) = v(:,end) + dr_gamma*omega;                            % update derivative of S-T curve
                             
% Call ST tracking controller
    q_hat = x_hat(1:2,end);                                                 % estimate of target trajectory                                              
    v_hat = x_hat(3:4,end);                                                 % estimate of target velocity
    
    pd_hat = q_hat  + r_gamma ;
    pd_dot_hat = v_hat + dr_gamma*omega;                                    % time derivative of pd                          
    pd_gamma = dr_gamma;                                                    % partial derivative of pd respect to gamma
    
    % Call ST tracking controller
    [Tracker.Input(:,end+1), gamma_dot(end+1), e_pos_hat, e_gamma] ... 
     = Tracking_Controller_2D(t,Ts,p(:,end),psi(end),pd_hat,pd_dot_hat,pd_gamma, TTC_type, gamma_dot(end),omega, gains);
 
    Tracker.Input(:,end) = max(min(Tracker.Input(:,end),Tracker.InputMax),Tracker.InputMin);          % saturate the linear speed and angular speed    
    
% Compute position error w.r.t. q - true target trajectory
    R_IB = [cos(psi(end)) -sin(psi(end));                                  % rotation matrix from I to B
            sin(psi(end))  cos(psi(end))];
    e_pos_q = R_IB'*(p(:,end) - pd(:,end)) - gains.epsilon; 

    E_Pursuit(end+1) = norm([e_pos_q ; e_gamma]);                           % update pursuit error
% Simulate range to target     
    range(end+1) = norm([p(:,end) - q(:,end);Target.Depth]) + 0.5*randn;
% Estimate the target state
    [x_hat(:,end+1),P_hat] = STST_EKF_PosVel(range(end),p(:,end),x_hat(:,end),P_hat,Ts,Target.Depth,t);
    P_save = [P_save P_hat];
    E_Loc(end+1) = norm(x(:,end)-x_hat(:,end));       % update localization error         
% Update tracker state
    Tracker.State(:,end+1) = update_vehicle(Tracker.State(:,end),Tracker.Input(:,end),Ts,t);  
    p = Tracker.State(1:2,:);                                      % tracker position
    psi = Tracker.State(3,:);                                        % tracker heading
% Update gamma
    gamma(end+1) = gamma(end) + Ts*gamma_dot(end);
end
% -------------------------------Save Data to Workspace ------------------
    p = p';
    q = q';
    x_hat = x_hat';
    pd = pd';
    
    save_to_base(1);
    animation_1vehicles
end
%% ======================= Auxilary functions =============================== 
function x_next=update_vehicle(x_current,input,Ts,t)
    input_robot.nSteps = 4;
    input_robot.Ts=Ts;
    input_robot.u=input;
    input_robot.x = x_current;
    output_robot = RK4_integrator(@vehicle, input_robot);
    x_next = output_robot.value;
end
% vehicle dynamics
function dx = vehicle(t,x,u)
    psi=x(3);
    ur=u(1);
    r=u(2);
    dx=[ur*cos(psi);ur*sin(psi);r];
end
