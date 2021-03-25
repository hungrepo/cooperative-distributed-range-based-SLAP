
% m=600;
m=10; 
N=round((size(p1,1)-1)/m);
t1=zeros(N,2);
t2=zeros(N,2);
t3=zeros(N,2);
t4=zeros(N,2);
t5=zeros(N,2);
t6=zeros(N,2);
t7=zeros(N,2);
t8=zeros(N,2);
t9=zeros(N,2);
t10=zeros(N,2);
t11=zeros(N,2);

t20=zeros(4,N*4);
t21=zeros(4,N*4);
t22=zeros(4,N*4);

t25=zeros(N,4);
t26=zeros(N,4);
t27=zeros(N,4);


t30=zeros(N,2);
t31=zeros(N,2);
t32=zeros(N,2);


k=1;
for i=0:N-1;

t1(i+1,:)=pd1(m*i+k,:);
t30(i+1,:)=pd1_hat(m*i+k,:);

t2(i+1,:)=p1(m*i+k,:); 
t3(i+1,:)=yaw1(m*i+k,:); 
        
t4(i+1,:)=pd2(m*i+k,:);
t31(i+1,:)=pd2_hat(m*i+k,:);

t5(i+1,:)=p2(m*i+k,:); 
t6(i+1,:)=yaw2(m*i+k,:); 

t7(i+1,:)=pd3(m*i+k,:);
t32(i+1,:)=pd3_hat(m*i+k,:);

t8(i+1,:)=p3(m*i+k,:); 
t9(i+1,:)=yaw3(m*i+k,:); 


t10(i+1,:)=Target_PosEst1(m*i+k,:);
t11(i+1,:)=Target_PosEst2(m*i+k,:);
t12(i+1,:)=Target_PosEst3(m*i+k,:);

t13(i+1,:)=Target_Pos(m*i+k,:);

%% covariance
t20(:,4*i+1:4*i+4)=P1_save(:,m*4*i+1:m*4*i+4);
t21(:,4*i+1:4*i+4)=P2_save(:,m*4*i+1:m*4*i+4);
t22(:,4*i+1:4*i+4)=P3_save(:,m*4*i+1:m*4*i+4);


t25(i+1,:)=Target.Id1.Est1(m*i+k,:);
t26(i+1,:)=Target.Id1.Est2(m*i+k,:);
t27(i+1,:)=Target.Id1.Est3(m*i+k,:);




end

pd1=t1;
p1=t2;
yaw1=t3;

pd2=t4;
p2=t5;
yaw2=t6;

pd3=t7;
p3=t8;
yaw3=t9;



Target_PosEst1=t10;
Target_PosEst2=t11;
Target_PosEst3=t12;

Target_Pos=t13;

%% Target estimated state
Target.Id1.Est1=t25;
Target.Id1.Est2=t26;
Target.Id1.Est3=t27;



%% covariance
P1_save=t20;
P2_save=t21;
P3_save=t22;

pd1_hat=t30;
pd2_hat=t31;
pd3_hat=t32;



