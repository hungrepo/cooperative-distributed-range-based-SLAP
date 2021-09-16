%% This code implements the distributed EKF with event triggered communication algorithm used in the paper

%   Cooperative distributed estimation and control of multiple autonomous vehicles 
%   for range-based underwater target localization and pursuit

%   Authors: Nguyen T. Hung, Francisco Rego, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/

%   
%% ======================================== DEKF with ETC ===================================================================
    
function [Tracker1, Tracker2, Tracker3] = DEKF_3V_3D_ETC(Tracker1, Tracker2, Tracker3, Target,Ts,t)

% Tuning parameters
    Q = 1e-4*diag([10 10 10 1 1 1]);
    W = inv(Q);
    R1 = 1;    R2 = 1;    R3 = 1;
    V1 = inv(R1);   V2 = inv(R2);   V3 = inv(R3);

% Target model
    F = [eye(3)   Ts*eye(3);
         zeros(3) eye(3)];


% Correct the prediction only when having new ranges - at every T_meas 
    T_meas = 2;
    if rem(t,T_meas)==0
% Local correction   
       % At Tracker 1
        Tracker1 = local_correction(Tracker1,Target,V1);
       % At Tracker 2
        Tracker2 = local_correction(Tracker2,Target,V2);
       % At Tracker 3 
        Tracker3 = local_correction(Tracker3,Target,V3);
% Call ETC mechanism
       % At Tracker1
        Tracker1 = ETC(Tracker1,t);
       % At Tracker2 
        Tracker2 = ETC(Tracker2,t);
       % At Tracker3 
        Tracker3 = ETC(Tracker3,t);
        
% Funsion Step (after receving data from the neighbors)
       % At Tracker 1 
        Tracker1 = fusion_ETC(Tracker1,Tracker3);
       % At Tracker 2 
        Tracker2 = fusion_ETC(Tracker2,Tracker1);
       % At Tracker 3
        Tracker3 = fusion_ETC(Tracker3,Tracker2);
    else
      % if having no range information, just keep predicted state and covariance  
        Tracker1.x_hat_cor = Tracker1.x_hat(:,end);
        Tracker2.x_hat_cor = Tracker2.x_hat(:,end);
        Tracker3.x_hat_cor = Tracker3.x_hat(:,end);        
        Tracker1.Omega_cor = inv(Tracker1.P_hat);
        Tracker2.Omega_cor = inv(Tracker2.P_hat);
        Tracker3.Omega_cor = inv(Tracker3.P_hat);
    end
% Predict corrected density 
    Tracker1 = prediction(Tracker1, F, W);
    Tracker2 = prediction(Tracker2, F, W);
    Tracker3 = prediction(Tracker3, F, W);
% Predicted reference density
    Tracker1 = predict_reference(Tracker1, F, Q);
    Tracker2 = predict_reference(Tracker2, F, Q);
    Tracker3 = predict_reference(Tracker3, F, Q);
end   
function Tracker = local_correction(Tracker,Target,V)
    q_hat = Tracker.x_hat(1:3,end);                                         % Current estimated position of target
    p = Tracker.p(:,end);                                                   % Current position of tracker
    h_hat = norm(q_hat-p);                                   % Current estimated range from tracker to target
    y = Tracker.range(end);                                                 % Current true range from tracker to target
    x_hat = Tracker.x_hat(:,end);
    Omega = inv(Tracker.P_hat);     
    z = Omega*x_hat;                                                       % Current informaiton vector
        
    C = [(q_hat - p)'/h_hat 0 0 0];        
    y_err = y - h_hat + C*x_hat;
         
    z_tilde = z + C'*V*y_err;                                      % will be transmited to neighbor, i.e. Tracker 3 
    Omega_tilde = Omega + C'*V*C;                                  % will be transmited to neighbor, i.e. Tracker 3
    
    P_tilde = inv(Omega_tilde);
    x_tilde = P_tilde*z_tilde;
% Store    
    Tracker.z_tilde = z_tilde;
    Tracker.Omega_tilde = Omega_tilde;

    Tracker.P_tilde = P_tilde;
    Tracker.x_tilde = x_tilde;
end
function Tracker = fusion_ETC(Tracker,neighbor)
    % For simplicity set all weight pij = 0.5 for all i,j
     if (neighbor.Com_DEKF(end) == 1)
        Tracker.z_cor = 0.5*Tracker.z_tilde + 0.5*neighbor.z_tilde;
        Tracker.Omega_cor = 0.5*Tracker.Omega_tilde + 0.5*neighbor.Omega_tilde;
     else
        Tracker.z_cor = Tracker.z_tilde; 
        Tracker.Omega_cor = Tracker.Omega_tilde;
     end
     Tracker.x_hat_cor = inv(Tracker.Omega_cor)*Tracker.z_cor;
end
function Tracker = prediction(Tracker,F,W)
    x_hat_cor = Tracker.x_hat_cor;
    Omega_cor = Tracker.Omega_cor;
   % predict based on the corrected density 
    x_hat_pre = F*x_hat_cor;
    Omega_pre = W - W*F/(Omega_cor + F'*W*F)*F'*W;
   
    Tracker.x_hat(:,end+1) = x_hat_pre;
    Tracker.P_hat = inv(Omega_pre);
    Tracker.P_save = [Tracker.P_save Tracker.P_hat];   
end    
function Tracker = ETC(Tracker,t)
    x_bar = Tracker.x_bar(:,end);
    Omega_bar = inv(Tracker.P_bar);
    x_tilde = Tracker.x_tilde;
    P_tilde = Tracker.P_tilde;
    Omega_tilde = inv(P_tilde);
  % compute triggering function
    c0 = 3;     c1 = 15;     c2 = 0.05;
    g = c0+c1*exp(-c2*t);        
    % compute KLD
    n = length(x_bar);
    KLD = trace(Omega_bar*P_tilde)+(x_bar-x_tilde)'*Omega_bar*(x_bar-x_tilde)...
                              +log(det(Omega_tilde)/det(Omega_bar))-n;
                          
    % check if we need to transmit or not
    if KLD >= g
        Com_DEKF = 1;                                                   % transmit
        Tracker.P_bar = P_tilde;                                        % reset reference density
        Tracker.x_bar(:,end) = x_tilde;
    else
        Com_DEKF = 0; 
    end
% store   
    Tracker.KLD(end+1) = KLD;
    Tracker.g(end+1) = g;
    Tracker.Com_DEKF(end+1) = Com_DEKF;
end            
function Tracker = predict_reference(Tracker,F, Q)
% predict reference density 
   
    x_bar = Tracker.x_bar(:,end);
    P_bar = Tracker.P_bar;
    
    x_bar_pre = F*x_bar;
    P_bar_pre = F*P_bar*F' + Q;
    
    Tracker.x_bar(:,end+1) = x_bar_pre;
    Tracker.P_bar = P_bar_pre;
end    