%% This code implements the distributed EKF algorithm used in the paper

%   Cooperative distributed estimation and control of multiple autonomous vehicles 
%   for range-based underwater target localization and pursuit

%   Authors: Nguyen T. Hung, Francisco Rego, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/

%   
%% ======================================== DEKF ===================================================================

function [Tracker1, Tracker2, Tracker3] = DEKF_3V_3D(Tracker1, Tracker2, Tracker3, Target,Ts,t)

% Tuning parameters
    Q = 1e-4*diag([10 10 10 1 1 1]);
    W = inv(Q);
    R1 = 1;    R2 = 1;    R3 = 1;
    V1 = inv(R1);   V2 = inv(R2);   V3 = inv(R3);

% Target model
F = [eye(3) Ts*eye(3);
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

% Funsion Step (after receving data from the neighbors)
       % At Tracker 1 
        Tracker1 = fusion(Tracker1,Tracker3);
       % At Tracker 2 
        Tracker2 = fusion(Tracker2,Tracker1);
       % At Tracker 3
        Tracker3 = fusion(Tracker3,Tracker2);
    else
      % if having no range information, just keep predicted state and covariance  
        Tracker1.x_hat_cor = Tracker1.x_hat(:,end);
        Tracker2.x_hat_cor = Tracker2.x_hat(:,end);
        Tracker3.x_hat_cor = Tracker3.x_hat(:,end);        
        Tracker1.Omega_cor = inv(Tracker1.P_hat);
        Tracker2.Omega_cor = inv(Tracker2.P_hat);
        Tracker3.Omega_cor = inv(Tracker3.P_hat);
    end
% Predict
    Tracker1 = prediction(Tracker1, F, W);
    Tracker2 = prediction(Tracker2, F, W);
    Tracker3 = prediction(Tracker3, F, W);
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
    
    Tracker.z_tilde = z_tilde;
    Tracker.Omega_tilde = Omega_tilde;
end
function Tracker = fusion(Tracker,neighbor)
    % For simplicity set all weight pij = 0.5 for all i,j
     Tracker.z_cor = 0.5*Tracker.z_tilde + 0.5*neighbor.z_tilde;
     Tracker.Omega_cor = 0.5*Tracker.Omega_tilde + 0.5*neighbor.Omega_tilde;
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