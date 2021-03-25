function [xhat1Out,xhat2Out,xhat3Out,P1Out,P2Out,P3Out,Com_Sig,d_lk] = DEKF_event(y1,y2,y3,p1,p2,p3,x1hat,x2hat,x3hat,P1,P2,P3,Ts,depth,t) 
% Calculate the Jacobians for the state and measurement equations
n=length(x1hat);
persistent x1_bar x2_bar x3_bar  Omega1_bar Omega2_bar Omega3_bar
    if t==0
        x1_bar=x1hat;              Omega1_bar=inv(P1);
        x2_bar=x2hat;              Omega2_bar=inv(P2);    
        x3_bar=x3hat;              Omega3_bar=inv(P3);
    end
q1_bar= Omega1_bar*x1_bar;
q2_bar= Omega2_bar*x2_bar;
q3_bar= Omega3_bar*x3_bar;

% triggering function    
c0=2; c1=5; c2=.1;

Omega1=inv(P1);
Omega2=inv(P2);
Omega3=inv(P3);

q1=Omega1*x1hat;
q2=Omega2*x2hat;
q3=Omega3*x3hat;

Q = 1e-4*diag([10 10 1 1]);
R1=10;
R2=10;
R3=10;
W=inv(Q);
V1=inv(R1);
V2=inv(R2);
V3=inv(R3);
A = [1 0 Ts 0;
     0 1 0  Ts;
     0 0 1  0;
     0 0 0  1];

pos1_hat=x1hat(1:2);
pos2_hat=x2hat(1:2);
pos3_hat=x3hat(1:2);
h1hat = norm([pos1_hat-p1;depth]);
h2hat = norm([pos2_hat-p2;depth]);
h3hat = norm([pos3_hat-p3;depth]);

 %% Correction
if rem(t,2)==0
    C1=[(pos1_hat-p1)'/h1hat 0 0];
    C2=[(pos2_hat-p2)'/h2hat 0 0];
    C3=[(pos3_hat-p3)'/h3hat 0 0];

    y1_err=y1-h1hat+C1*x1hat;
    y2_err=y2-h2hat+C2*x2hat;
    y3_err=y3-h3hat+C3*x3hat;

    delta_q1=C1'*V1*y1_err;
    delta_q2=C2'*V2*y2_err;
    delta_q3=C3'*V3*y3_err;
    
    delta_Omega1=C1'*V1*C1;
    delta_Omega2=C2'*V2*C2;
    delta_Omega3=C3'*V3*C3;
else 
    delta_q1=zeros(n,1);
    delta_q2=zeros(n,1);
    delta_q3=zeros(n,1);
    
    delta_Omega1=zeros(n,n);
    delta_Omega2=zeros(n,n);
    delta_Omega3=zeros(n,n);
end


% correct    
    q1_cor=q1+delta_q1;
    q2_cor=q2+delta_q2;
    q3_cor=q3+delta_q3;
    
    Omega1_cor=Omega1+delta_Omega1;
    Omega2_cor=Omega2+delta_Omega2;
    Omega3_cor=Omega3+delta_Omega3;  
    
    x1hat_cor=inv(Omega1_cor)*q1_cor;
    x2hat_cor=inv(Omega2_cor)*q2_cor;
    x3hat_cor=inv(Omega3_cor)*q3_cor;
 %% Fusiion   
Com_mode=2;

if Com_mode==2     
  % Update control input
%     q1_fus=(q1+q3_bar)/2;
%     q2_fus=(q2+q1_bar)/2;
%     q3_fus=(q3+q2_bar)/2;
%     Omega1_fus=(Omega1+Omega3_bar)/2;
%     Omega2_fus=(Omega2+Omega1_bar)/2;
%     Omega3_fus=(Omega3+Omega2_bar)/2;
   % Check communication
             h=c0+c1*exp(-c2*t(end));
             e(1)=trace(Omega1_bar*inv(Omega1_cor))+(x1_bar-x1hat_cor)'*Omega1_bar*(x1_bar-x1hat_cor)+log(det(Omega1_cor)/det(Omega1_bar))-n;
             e(2)=trace(Omega2_bar*inv(Omega2_cor))+(x2_bar-x2hat_cor)'*Omega2_bar*(x2_bar-x2hat_cor)+log(det(Omega2_cor)/det(Omega2_bar))-n;
             e(3)=trace(Omega3_bar*inv(Omega3_cor))+(x3_bar-x3hat_cor)'*Omega3_bar*(x3_bar-x3hat_cor)+log(det(Omega3_cor)/det(Omega3_bar))-n;
             d_lk=e;

           delta(1)=e(1)-h; 
           delta(2)=e(2)-h;
           delta(3)=e(3)-h;
  % Broadcast signal
            if  delta(1)>= 0
                Com_Sig(1)=1; q1_bar=q1_cor; Omega1_bar=Omega1_cor;  x1_bar=inv(Omega1_bar)*q1_bar;
                q2_fuse=(q2_cor+q1_bar)/2;
                Omega2_fuse=(Omega2_cor+Omega1_bar)/2;
            else
                Com_Sig(1)=0;
                q2_fuse=q2_cor;
                Omega2_fuse=Omega2_cor;
            end
            if  delta(2)>= 0
                Com_Sig(2)=1; q2_bar=q2_cor; Omega2_bar=Omega2_cor;  x2_bar=inv(Omega2_bar)*q2_bar;
                  q3_fuse=(q3_cor+q2_bar)/2;
                  Omega3_fuse=(Omega3_cor+Omega2_bar)/2;
            else
                Com_Sig(2)=0;
                q3_fuse=q3_cor;
                Omega3_fuse=Omega3_cor;
            end
            if  delta(3)>= 0
                Com_Sig(3)=1; q3_bar=q3_cor; Omega3_bar=Omega3_cor;  x3_bar=inv(Omega3_bar)*q3_bar;
                    q1_fuse=(q1_cor+q3_bar)/2;
                    Omega1_fuse=(Omega1_cor+Omega3_bar)/2;
            else
                Com_Sig(3)=0;
                q1_fuse=q1_cor;
                Omega1_fuse=Omega1_cor;
            end
end
  
  xhat1_cor=inv(Omega1_fuse)*q1_fuse;
  xhat2_cor=inv(Omega2_fuse)*q2_fuse;
  xhat3_cor=inv(Omega3_fuse)*q3_fuse;
  Omega1_cor=Omega1_fuse;
  Omega2_cor=Omega2_fuse;
  Omega3_cor=Omega3_fuse;
  
  
%   Com_Sig=[0;0;0];

%% Predict
xhat1_pre = A*xhat1_cor;
Omega1_pre=W-W*A/(Omega1_cor+A'*W*A)*A'*W;

xhat2_pre = A*xhat2_cor;
Omega2_pre=W-W*A/(Omega2_cor+A'*W*A)*A'*W;

xhat3_pre = A*xhat3_cor;
Omega3_pre=W-W*A/(Omega3_cor+A'*W*A)*A'*W;

% Predict in event triggered
x1_bar= A*x1_bar;
Omega1_bar=W-W*A/(Omega1_bar+A'*W*A)*A'*W;

x2_bar= A*x2_bar;
Omega2_bar=W-W*A/(Omega2_bar+A'*W*A)*A'*W;

x3_bar= A*x3_bar;
Omega3_bar=W-W*A/(Omega3_bar+A'*W*A)*A'*W;



xhat1Out=xhat1_pre;
P1Out=inv(Omega1_pre);
xhat2Out=xhat2_pre;
P2Out=inv(Omega2_pre);
xhat3Out=xhat3_pre;
P3Out=inv(Omega3_pre);

