function [qhatOut,POut] = MTST_EKF_Pos(r1,r2,p1,p2,qhat,uk,P,Ts,depth)
% Covariance matrixes
Q = 1e-4*diag([10 10]);
R = .1;
% Jacobian
F = [1 0;0 1];
r1hat = norm([qhat-p1;depth]);
r2hat = norm([qhat-p2;depth]);

H = [(qhat-p1)'/norm(qhat-p1);
     (qhat-p2)'/norm(qhat-p2)];
% Predict
qhat = F*qhat+Ts*uk;
P = F*P*F' + Q;
% Calculate the Kalman gain
K = P*H'/(H*P*H' + R);
% Calculate the measurement residual
resid = [r1 - r1hat; r2-r2hat];
% Update the state and covariance estimates
qhat = qhat + K*resid;
P = (eye(size(K,1))-K*H)*P;
% Post the results
qhatOut = qhat;
POut=P;