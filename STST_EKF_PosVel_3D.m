function [xhatOut,POut] = STST_EKF_PosVel_3D(r,p,xhat,P,Ts,t) 
% Calculate the Jacobians for the state and measurement equations

% Q = 1e-4*diag([.1 .1 .1 .1]);
Q = 1e-4*diag([10 10 10 1 1 1]);

R = 1;

F = [eye(3) Ts*eye(3);
     zeros(3) eye(3)];
qhat=xhat(1:3);
rhat = norm(qhat-p);
H = [(qhat-p)'/norm(qhat-p) 0 0 0] ;
% Predict
xhat_pre = F*xhat;
%% -------------
% A = [zeros(3)  eye(3);
%      zeros(3)  zeros(3)]; % Some arbitrary matrix we will use
% odefun = @(t,y) deriv(t,y,A);  % Anonymous derivative function with A
% tspan = [0 Ts];
% [T,M] = ode45(odefun,tspan,P(:));  % Pass in column vector initial value
% P_pre = reshape(M.',6,6,[]);  % Reshape the output as a sequence of 2x2 matrices
% P_pre = P_pre(:,:,end); 
%% ----------------
P_pre = F*P*F' + Q;
%P_pre=update_P(P,F,Ts);

% tspan = [0 Ts];
% [t,y] = ode45(@(t,y) F*y+y*F'+Q, tspan, P);

% Calculate the Kalman gain
K = P_pre*H'/(H*P_pre*H' + R);
% Calculate the measurement residual
resid = r - rhat;
% Update the state and covariance estimates
if rem(t,2)==0
    xhat_corr = xhat_pre + K*resid;
    P_corr = (eye(size(K,1))-K*H)*P_pre;
else
    xhat_corr=xhat_pre;
    P_corr=P_pre;
end
% Post the results
xhatOut = xhat_corr;
POut=P_corr;
end 
% function dy = deriv(t,y,A)
% Q=1e-4*diag([10 10 10 1 1 1]);
% F=A;
% P = reshape(y,size(A));  % Reshape input y into matrix
% FA = F*P+P*F'+Q;  % Do the matrix multiply
% dy = FA(:);  % Reshape output as a column vector
% end
