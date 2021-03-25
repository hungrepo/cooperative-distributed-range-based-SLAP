
%% This code is used to generate simulation results for the paper entitled 

%   Cooperative distributed estimation and control of multiple autonomous vehicles 
%   for range-based underwater target localization and pursuit”

%   Authors: Nguyen T. Hung, Francisco Rego, Antonio Pascoal, Institute System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/

%%  Description: tracking S-T curve, a hybrid trajectory tracking and path following
%   Inputs:
%       t:                  current time
%       Ts:                 sampling time
%       p:                  vehicle position in 2D
%       psi:                vehicle heading
%       pd:                 desired S-T curve to be track   
%       pd_dot:             time derivative of pd
%       pd_gamma:           partial derivative of pd respect to
%                           the path parameter gamma
%       controller_type:    TypeI or TypeII
%       vd:                 desired speed profile for gamma_dot
%       gains:              Controller tuning parameter
%       
%   Outputs:
%       u:                  vehicle linear and angular speed
%       gamma_dot_new:      desired derivative of gamma
%       e_pos:              position error in the Body frame - this is
%                           optional just for mornitoring
%       e_gamma:            the error on the derivative of gamma - this is
%                           optional, just for mornitoring
%% =========================================================================================================================
function [u, gamma_dot_new, e_pos, e_gamma] = Tracking_Controller_3D(t,Ts,p,eta,pd,pd_dot,pd_gamma,controller_type, gamma_dot_old, vd, gains)
persistent epsilon Delta_bar K kz gamma_ddot_max gamma_ddot_min;
if (t == 0)
% Set controller parameters for the first time
    epsilon = gains.epsilon;
    Delta_bar = gains.Delta_bar;   
    K = gains.K;   
    kz = gains.kz;         
    gamma_ddot_max = gains.gamma_ddot_max;
    gamma_ddot_min = gains.gamma_ddot_min;

end
R_IB = Rotation_matrix(eta(1),eta(2),eta(3));                               % yaw, pitch, roll;
e_pos = R_IB'*(p-pd) - epsilon;    
u = Delta_bar*(R_IB'*pd_dot - K*e_pos);                                     % compute linear and angular speeds
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

