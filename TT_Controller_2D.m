
%% This code is used to generate simulation results for the paper entitled 

%   Cooperative distributed estimation and control of multiple autonomous vehicles 
%   for range-based underwater target localization and pursuit”

%   Authors: Nguyen T. Hung, Francisco Rego, Antonio Pascoal, Institute System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/
%%  Description: tracking S-T curve, a hybrid trajectory tracking and path following
%   Inputs:
%       vehicle_state.p:    vehicle position in 2D
%       vehicle_state.yaw:  heading of vehicle
%       trajectory.pd:      the S-T curve to be track    
%       trajectory.d_pd:    time derivative of the S-T curve
%       pd_gamma:           partial derivative of the S-T curve respect to
%                           the path parameter gamma
%   Output:
%       u:                  vehicle linear and angular speed
%       gamma_dot_new:      desired derivative of gamma
%       e_pos:              position error in the Body frame - this is
%                           optional just for mornitoring
%       e_gamma:            the error on the derivative of gamma - this is
%                           optional, just for mornitoring
% =========================================================================================================================
function [u, gamma_dot_new, e_pos, e_gamma] = ST_Tracking_2D(t,Ts,vehicle_state, trajectory, controller_type, gamma_dot_old, vd)
p = vehicle_state.pos;
yaw = vehicle_state.yaw;
pd = trajectory.pd;                                         % desired trajectory to be track
d_pd = trajectory.d_pd;                                     % 
pd_gamma = trajectory.pd_gamma;
persistent epsilon Delta_inv K kz gamma_ddot_max gamma_ddot_min;
if (t == 0)
% Set controller parameters for the first time
    delta = -1;
    epsilon = [delta; 0];
    Delta = [1 0; 
             0 -delta];
    Delta_inv = inv(Delta);   
    K = diag([0.4, 0.2]);   
    kz = 500;         
    gamma_ddot_max =  0.01;
    gamma_ddot_min = -0.001;

end
R_IB = [cos(yaw) -sin(yaw);                     % rotation matrix from I to B
        sin(yaw)  cos(yaw)];
e_pos = R_IB'*(p-pd) - epsilon;    
u = Delta_inv*(R_IB'*d_pd - K*e_pos);           % compute linear and angular speeds
if strcmp(controller_type, 'TypeI')
    gamma_dot_new = vd;
    e_gamma = 0;
else
    % go with controller TypeII
    e_gamma = gamma_dot_old - vd;
    gamma_ddot = -kz*e_gamma + e_pos'*R_IB'*pd_gamma;
    gamma_ddot = max(min(gamma_ddot,gamma_ddot_max),gamma_ddot_min);
    gamma_dot_new = gamma_dot_old + Ts*gamma_ddot;
end
end

