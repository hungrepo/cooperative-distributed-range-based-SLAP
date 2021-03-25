
%% This code is used to generate simulation results for one tracker- one target in the paper entitled: 

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
    N = 1;                                                                  % number of tracker used
    omega_bar = 0.1;                                                       % desired angular rate encircling about target  
% Initialize the tracker's position and orietation
    p = [55;-60; 25];                                                            % tracker position    
    eta = [pi;0;0];                                                                % tracker heading - yaw
    Tracker.State = [p; eta];
    Tracker.Input = [];                                                     % linear and angular speeds
    Tracker.InputMin = [ 0 ; -1; -1; -1];                                          % maximum linear and angular speeds
    Tracker.InputMax = [10 ;  1;  1;  0.4];                                          % minimum linear and angular speeds
% Initialize the target trajectory
    Target.Depth = 0;
% The Spatial-Temporal (ST) cure for the vehicle to track       
    % Spatial part     
        gamma =  5.497787143782138;   
        gamma_dot = 0.0;
        rx = 80;   ry = 80;   rz = 50;   phi = 0;        
% Initialize the estimate of target's trajectory       
    q_hat = [-30; 12; -25];                                                     % Initial estimate of target position
    v_hat = [0.2; 0.2; -0.1];                                                    % Initial estimate of target velocity 
    x_hat = [q_hat; v_hat];                                                 % Initial estimate of target state
    P_hat = diag([100,100,100,1,1,0.1]);                                       % Initial estimate of covariance
    P_save = P_hat;
% Set gains of the tracking controller 
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
    q(:,end+1) = [30*sin(0.01*t);   .1*t;   -0.5*t ];                             % target position
    v(:,end+1) = [0.3*cos(0.01*t);  .1  ;   -0.5 ];                             % target velocity
    x(:,end+1) = [q(:,end); v(:,end)];                                      % target state

% Update desired S-T curve
    r_gamma = [rx*cos(gamma(end) + phi);                                      % spatial path (circle about target)
               ry*sin(gamma(end) + phi)
               rz*sin(0.5*gamma(end) + phi)]; 
    dr_gamma = [-rx*sin(gamma(end) + phi);                                    % partial derivative of r respect to gamma
                 ry*cos(gamma(end) + phi)
              .5*rz*cos(.5*gamma(end) + phi)];  
    pd(:,end+1) = q(:,end) + r_gamma;                                       % update S-T curve
    pd_dot(:,end+1) = v(:,end) + dr_gamma*omega;                            % update derivative of S-T curve
                             
% Call ST tracking controller
    q_hat = x_hat(1:3,end);                                                 % estimate of target trajectory                                              
    v_hat = x_hat(4:6,end);                                                 % estimate of target velocity
    
    pd_hat = q_hat  + r_gamma ;
    pd_dot_hat = v_hat + dr_gamma*omega;                                    % time derivative of pd                          
    pd_gamma = dr_gamma;                                                    % partial derivative of pd respect to gamma
    
    % Call ST tracking controller
    [Tracker.Input(:,end+1), gamma_dot(end+1), e_pos_hat, e_gamma] ... 
     = Tracking_Controller_3D(t,Ts,p(:,end),eta(:,end),pd_hat,pd_dot_hat,pd_gamma, TTC_type, gamma_dot(end),omega, gains);
 
    Tracker.Input(:,end) = max(min(Tracker.Input(:,end),Tracker.InputMax),Tracker.InputMin);          % saturate the linear speed and angular speed    
    
% Compute position error w.r.t. q - true target trajectory
    R_IB = Rotation_matrix(eta(1),eta(2),eta(3));                               % yaw, pitch, roll;
    e_pos_q = R_IB'*(p(:,end) - pd(:,end)) - gains.epsilon; 

    E_Pursuit(end+1) = norm([e_pos_q ; e_gamma]);                           % update pursuit error
% Simulate range to target     
    range(end+1) = norm([p(:,end) - q(:,end)]) + 0.5*randn;
% Estimate the target state
    [x_hat(:,end+1),P_hat] = STST_EKF_PosVel_3D(range(end),p(:,end),x_hat(:,end),P_hat,Ts,t);
    P_save = [P_save P_hat];
    E_Loc(end+1) = norm(x(:,end)-x_hat(:,end));       % update localization error         
% Update tracker state
    Tracker.State(:,end+1) = update_vehicle(Tracker.State(:,end),Tracker.Input(:,end),Ts,t);  
    p = Tracker.State(1:3,:);                                      % tracker position
    eta = Tracker.State(4:6,:);                                        % tracker heading
% Update gamma
    gamma(end+1) = gamma(end) + Ts*gamma_dot(end);
end
% -------------------------------Save Data to Workspace ------------------
    p = p';
    q = q';
    x_hat = x_hat';
    pd = pd';
    Tracker.State = Tracker.State';
    
    save_to_base(1);
    animation_1vehicles_3D
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
