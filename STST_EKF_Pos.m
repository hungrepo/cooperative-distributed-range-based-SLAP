function [qhatOut,POut] = STST_EKF_Pos(r1,p1,qhat,uk,P,Ts,depth)
% Covariance matrixes
Q = 1e-4*diag([10 10]);
R = 10;
% Jacobian
F = [1 0;0 1];
r1hat = norm([qhat-p1;depth]);
 
H = (qhat-p1)'/r1hat;
% Predict
qhat = F*qhat+Ts*uk;
P = F*P*F' + Q;
% Calculate the Kalman gain
K = P*H'/(H*P*H' + R);
% Calculate the measurement residual
resid = r1 - r1hat;
% Update the state and covariance estimates
qhat = qhat + K*resid;
P = (eye(size(K,1))-K*H)*P;
% Post the results
qhatOut = qhat;
POut=P;