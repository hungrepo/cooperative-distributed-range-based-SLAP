
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


k=1;
for i=0:N-1;

t1(i+1,:)=pd1(m*i+k,:);
t2(i+1,:)=p1(m*i+k,:); 
t3(i+1,:)=yaw1(m*i+k,:); 
 
t20(:,4*i+1:4*i+4)=P1_save(:,m*4*i+1:m*4*i+4);


t10(i+1,:)=Target_PosEst1(m*i+k,:);


t13(i+1,:)=Target_Pos(m*i+k,:);
t25(i+1,:)=Target.Id1.Est1(m*i+k,:);

t40(i+1,:)=pd1_hat(m*i+k,:);
end

pd1=t1;
p1=t2;
yaw1=t3;

Target_PosEst1=t10;
 
P1_save=t20;

Target_Pos=t13;

Target.Id1.Est1=t25;
pd1_hat=t40;