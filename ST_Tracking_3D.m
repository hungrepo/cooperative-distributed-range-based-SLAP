function Tracker = ST_Tracking_3D(Tracker, Target, t, Ts, TTC_type, gains)
gamma = Tracker.gamma;
gamma_dot = Tracker.gamma_dot;
p = Tracker.p;
eta = Tracker.eta;
pd = Tracker.pd;
pd_dot = Tracker.pd_dot;
x_hat = Tracker.x_hat;
q = Target.q;
v = Target.v;
omega = Tracker.omega;

rx = Tracker.rx;
ry = Tracker.ry;
rz = Tracker.rz;
phi = Tracker.phi;

roll  =  Tracker.eta(1,end);
pitch =  Tracker.eta(2,end);
yaw  =  Tracker.eta(3,end);



E_Pursuit = Tracker.E_Pursuit;


% Update desired S-T curve
    r_gamma = [rx*cos(gamma(end) + phi);                                      % spatial path (circle about target)
               ry*sin(gamma(end) + phi);
               rz*sin(0.5*gamma(end) + phi)]; 
    dr_gamma = [ -rx*sin(gamma(end) + phi);                                    % partial derivative of r respect to gamma
                  ry*cos(gamma(end) + phi);
               .5*rz*cos(.5*gamma(end) + phi)];  
    pd(:,end+1) = q(:,end) + r_gamma;                                       % update S-T curve
    pd_dot(:,end+1) = v(:,end) + dr_gamma*omega(end);                            % update derivative of S-T curve
                             
% Call ST tracking controller
    q_hat = x_hat(1:3,end);                                                 % estimate of target trajectory                                              
    v_hat = x_hat(4:6,end);                                                 % estimate of target velocity
    
    pd_hat = q_hat  + r_gamma ;
    pd_dot_hat = v_hat + dr_gamma*omega(end);                                    % time derivative of pd                          
    pd_gamma = dr_gamma;                                                    % partial derivative of pd respect to gamma
    
    % Call ST tracking controller
    [Tracker.Input(:,end+1), gamma_dot(end+1), e_pos_hat, e_gamma] ... 
     = Tracking_Controller_3D(t,Ts,p(:,end),eta(:,end),pd_hat,pd_dot_hat,pd_gamma, TTC_type, gamma_dot(end),omega(end), gains);
 
    Tracker.Input(:,end) = max(min(Tracker.Input(:,end),Tracker.InputMax),Tracker.InputMin);          % saturate the linear speed and angular speed    
    
% Compute position error w.r.t. q - true target trajectory
    R_IB =  Rotation_matrix(roll,pitch,yaw);       

    e_pos_q = R_IB'*(p(:,end) - pd(:,end)) - gains.epsilon; 

    E_Pursuit(end+1) = norm([e_pos_q ; e_gamma]);                           % update pursuit error
% ========================================================
    Tracker.pd = pd;
    Tracker.pd_dot = pd_dot;
    Tracker.gamma = gamma;
    Tracker.gamma_dot = gamma_dot;
    Tracker.E_Pursuit = E_Pursuit;
    
end    




