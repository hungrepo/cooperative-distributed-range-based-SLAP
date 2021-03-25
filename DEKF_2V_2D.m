function [xhat1Out,xhat2Out,P1Out,P2Out] = DEKF2(y1,y2,p1,p2,x1hat,x2hat,P1,P2,Ts,depth,t) 
% Calculate the Jacobians for the state and measurement equations
Omega1=inv(P1);
Omega2=inv(P2);

q1=Omega1*x1hat;
q2=Omega2*x2hat;

Q = 1e-4*diag([10 10 1 1]);
R1=10;
R2=10;
W=inv(Q);
V1=inv(R1);
V2=inv(R2);
A = [1 0 Ts 0;
     0 1 0  Ts;
     0 0 1  0;
     0 0 0  1];

pos1_hat=x1hat(1:2);
pos2_hat=x2hat(1:2);
h1hat = norm([pos1_hat-p1;depth]);
h2hat = norm([pos2_hat-p2;depth]);

 %% Correction
if rem(t,2)==0
    C1=[(pos1_hat-p1)'/h1hat 0 0];
    C2=[(pos2_hat-p2)'/h2hat 0 0];

    y1_err=y1-h1hat+C1*x1hat;
    y2_err=y2-h2hat+C2*x2hat;

    delta_q1=C1'*V1*y1_err;
    delta_q2=C2'*V2*y2_err;
    
    delta_Omega1=C1'*V1*C1;
    delta_Omega2=C2'*V2*C2;
% Fusion via communication
    q1_fus=(q1+q2)/2;
    q2_fus=(q2+q1)/2;
    Omega1_fus=(Omega1+Omega2)/2;
    Omega2_fus=(Omega2+Omega1)/2;
    
    delta_q1_fus=(delta_q1+delta_q2)/2;
    delta_q2_fus=(delta_q2+delta_q1)/2;

    delta_Omega1_fus=(delta_Omega1 + delta_Omega2)/2;
    delta_Omega2_fus=(delta_Omega2 + delta_Omega1)/2;
 
% correct    
    q1_cor=q1_fus+delta_q1_fus;
    q2_cor=q2_fus+delta_q2_fus;
    
    Omega1_cor=Omega1_fus+delta_Omega1_fus;
    Omega2_cor=Omega2_fus+delta_Omega2_fus;
 
  xhat1_cor=inv(Omega1_cor)*q1_cor;
  xhat2_cor=inv(Omega2_cor)*q2_cor;
else
    xhat1_cor=x1hat;
    xhat2_cor=x2hat;
    Omega1_cor=Omega1;
    Omega2_cor=Omega2;
end
%% Predict
xhat1_pre = A*xhat1_cor;
Omega1_pre=W-W*A/(Omega1_cor+A'*W*A)*A'*W;

xhat2_pre = A*xhat2_cor;
Omega2_pre=W-W*A/(Omega2_cor+A'*W*A)*A'*W;


xhat1Out=xhat1_pre;
P1Out=inv(Omega1_pre);
xhat2Out=xhat2_pre;
P2Out=inv(Omega2_pre);


