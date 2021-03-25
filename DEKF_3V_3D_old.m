function [xhat1Out,xhat2Out,xhat3Out,P1Out,P2Out,P3Out] = DEKF_3vehicle_3D(y1,y2,y3,p1,p2,p3,x1hat,x2hat,x3hat,P1,P2,P3,Ts,t) 
% Calculate the Jacobians for the state and measurement equations
Omega1=inv(P1);
Omega2=inv(P2);
Omega3=inv(P3);

q1=Omega1*x1hat;
q2=Omega2*x2hat;
q3=Omega3*x3hat;

Q = 1e-4*diag([10 10 10 1 1 1]);
R1=1;
R2=1;
R3=1;
W=inv(Q);
V1=inv(R1);
V2=inv(R2);
V3=inv(R3);

A = [eye(3) Ts*eye(3);
     zeros(3) eye(3)];

pos1_hat=x1hat(1:3);
pos2_hat=x2hat(1:3);
pos3_hat=x3hat(1:3);
h1hat = norm(pos1_hat-p1);
h2hat = norm(pos2_hat-p2);
h3hat = norm(pos3_hat-p3);

 %% Correction
if rem(t,2)==0
    C1=[(pos1_hat-p1)'/h1hat 0 0 0];
    C2=[(pos2_hat-p2)'/h2hat 0 0 0];
    C3=[(pos3_hat-p3)'/h3hat 0 0 0];

    y1_err=y1-h1hat+C1*x1hat;
    y2_err=y2-h2hat+C2*x2hat;
    y3_err=y3-h3hat+C3*x3hat;

    delta_q1=C1'*V1*y1_err;
    delta_q2=C2'*V2*y2_err;
    delta_q3=C3'*V3*y3_err;
    
    delta_Omega1=C1'*V1*C1;
    delta_Omega2=C2'*V2*C2;
    delta_Omega3=C3'*V3*C3;
% Fusion via communication
    q1_fus=(q1+q3)/2;
    q2_fus=(q2+q1)/2;
    q3_fus=(q3+q2)/2;
    Omega1_fus=(Omega1+Omega3)/2;
    Omega2_fus=(Omega2+Omega1)/2;
    Omega3_fus=(Omega3+Omega2)/2;
    
    delta_q1_fus=(delta_q1+delta_q3)/2;
    delta_q2_fus=(delta_q2+delta_q1)/2;
    delta_q3_fus=(delta_q3+delta_q2)/2;

    delta_Omega1_fus=(delta_Omega1 + delta_Omega3)/2;
    delta_Omega2_fus=(delta_Omega2 + delta_Omega1)/2;
    delta_Omega3_fus=(delta_Omega3 + delta_Omega2)/2;
 
% correct    
    q1_cor=q1_fus+delta_q1_fus;
    q2_cor=q2_fus+delta_q2_fus;
    q3_cor=q3_fus+delta_q3_fus;
    
    Omega1_cor=Omega1_fus+delta_Omega1_fus;
    Omega2_cor=Omega2_fus+delta_Omega2_fus;
    Omega3_cor=Omega3_fus+delta_Omega3_fus;
 
  xhat1_cor=inv(Omega1_cor)*q1_cor;
  xhat2_cor=inv(Omega2_cor)*q2_cor;
  xhat3_cor=inv(Omega3_cor)*q3_cor;  
else
    xhat1_cor=x1hat;
    xhat2_cor=x2hat;
    xhat3_cor=x3hat;
    Omega1_cor=Omega1;
    Omega2_cor=Omega2;
    Omega3_cor=Omega3;
end
%% Predict
xhat1_pre = A*xhat1_cor;
Omega1_pre=W-W*A/(Omega1_cor+A'*W*A)*A'*W;

xhat2_pre = A*xhat2_cor;
Omega2_pre=W-W*A/(Omega2_cor+A'*W*A)*A'*W;

xhat3_pre = A*xhat3_cor;
Omega3_pre=W-W*A/(Omega3_cor+A'*W*A)*A'*W;

xhat1Out=xhat1_pre;
P1Out=inv(Omega1_pre);
xhat2Out=xhat2_pre;
P2Out=inv(Omega2_pre);
xhat3Out=xhat3_pre;
P3Out=inv(Omega3_pre);

