function upf = TT_Controller_2D(t,Ts,vehicle, trajectory, controller_type, gamma_dot, omega_bar)
p = vehicle.pos;
yaw = vehicle.yaw;
pd = trajectory.pd;
d_pd = trajectory.d_pd;
pd_gamma = trajectory.pd_gamma;
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
    gamma_ddot_min = -0.01;

end
R_IB = [cos(yaw) -sin(yaw);                 % rotation matrix from I to B
        sin(yaw)  cos(yaw)];
e_pos = R_IB'*(p-pd) - epsilon;    
u = Delta_inv*(Rot'*d_pd - K*e_pos);        % compute linear and angular speeds
if (controller_type == TypeI)
    gamma_dot_new = omega_bar;
else
    e_gamma = gamma_dot - omega_bar;
    gamma_ddot = -kz*e_gamma + e_pos'*R_IB'*pd_gamma;
    gamma_ddot = max(min(gamma_ddot,gamma_ddot_max),gamma_ddot_min);
    gamma_dot_new = gamma_dot + Ts*gamma_ddot;
end
upf = [u;gamma_dot_new];
end

