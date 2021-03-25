function [xhatOut,POut] = MTST_EKF_speed(r1,r2,p1,p2,xhat,P,Ts,depth,t) 
% Calculate the Jacobians for the state and measurement equations

Q = 1e-4*diag([10 10 1 1]);
R = [10 0; 
     0   10];

F = [1 0 Ts 0;
     0 1 0  Ts;
     0 0 1  0;
     0 0 0  1];
qhat=xhat(1:2);
r1hat = norm([qhat-p1;depth]);
r2hat = norm([qhat-p2;depth]);
H = [(qhat-p1)'/r1hat 0 0;
     (qhat-p2)'/r2hat 0 0];
% Predict
xhat = F*xhat;
P = F*P*F' + Q;
% Calculate the Kalman gain
K = P*H'/(H*P*H' + R);
% Calculate the measurement residual
resid = [r1 - r1hat;r2-r2hat];
% Update the state and covariance estimates
if rem(t,2)==0
xhat = xhat + K*resid;
P = (eye(size(K,1))-K*H)*P;
end
% Post the results
xhatOut = xhat;
POut=P;