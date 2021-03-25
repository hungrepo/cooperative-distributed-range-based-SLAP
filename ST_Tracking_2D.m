function Tracker = ST_Tracking_2D(Tracker, Target, t, Ts, TTC_type, gains)
gamma = Tracker.gamma;
gamma_dot = Tracker.gamma_dot;
p = Tracker.p;
psi = Tracker.psi;
pd = Tracker.pd;
pd_dot = Tracker.pd_dot;
x_hat = Tracker.x_hat;
q = Target.q;
v = Target.v;
omega = Tracker.omega;

rx = Tracker.rx;
ry = Tracker.ry;
phi = Tracker.phi;

E_Pursuit = Tracker.E_Pursuit;


% Update desired S-T curve
    r_gamma = [rx*cos(gamma(end) + phi);                                      % spatial path (circle about target)
               ry*sin(gamma(end) + phi)]; 
    dr_gamma = [-rx*sin(gamma(end) + phi);                                    % partial derivative of r respect to gamma
                 ry*cos(gamma(end) + phi)];  
    pd(:,end+1) = q(:,end) + r_gamma;                                       % update S-T curve
    pd_dot(:,end+1) = v(:,end) + dr_gamma*omega(end);                            % update derivative of S-T curve
                             
% Call ST tracking controller
    q_hat = x_hat(1:2,end);                                                 % estimate of target trajectory                                              
    v_hat = x_hat(3:4,end);                                                 % estimate of target velocity
    
    pd_hat = q_hat  + r_gamma ;
    pd_dot_hat = v_hat + dr_gamma*omega(end);                                    % time derivative of pd                          
    pd_gamma = dr_gamma;                                                    % partial derivative of pd respect to gamma
    
    % Call ST tracking controller
    [Tracker.Input(:,end+1), gamma_dot(end+1), e_pos_hat, e_gamma] ... 
     = Tracking_Controller_2D(t,Ts,p(:,end),psi(end),pd_hat,pd_dot_hat,pd_gamma, TTC_type, gamma_dot(end),omega(end), gains);
 
    Tracker.Input(:,end) = max(min(Tracker.Input(:,end),Tracker.InputMax),Tracker.InputMin);          % saturate the linear speed and angular speed    
    
% Compute position error w.r.t. q - true target trajectory
    R_IB = [cos(psi(end)) -sin(psi(end));                                  % rotation matrix from I to B
            sin(psi(end))  cos(psi(end))];
    e_pos_q = R_IB'*(p(:,end) - pd(:,end)) - gains.epsilon; 

    E_Pursuit(end+1) = norm([e_pos_q ; e_gamma]);                           % update pursuit error
% ========================================================
    Tracker.pd = pd;
    Tracker.pd_dot = pd_dot;
    Tracker.gamma = gamma;
    Tracker.gamma_dot = gamma_dot;
    Tracker.E_Pursuit = E_Pursuit;
    
end    




