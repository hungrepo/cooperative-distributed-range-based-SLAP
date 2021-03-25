% clear all;
close all;
AUV_COL33= [0.8 0.8 0]; % yellow
AUV_COL22= [0.1 0.1 0]; % black
AUV_COL11= [0.9 0.2 0]; % red


M=200;
Sigma1_0=P1_save(:,1: 4);
mu1=Target.Id1.Est1(1,:);
Data_Point1_0 = mvnrnd(mu1',(Sigma1_0+Sigma1_0.')/2, M);

    q1_hat1_0=[mu1(2),mu1(1)];
    covxx1=Sigma1_0(1,1);
    covyy1=Sigma1_0(2,2);
    Sigma1_0(1,1)=covyy1;
    Sigma1_0(2,2)=covxx1; 
    
        SigmaA=zeros(3,3);
    SigmaA(1:2,1:2)=Sigma1_0(1:2,1:2);

if N==2
Sigma2_0=P2_save(:,1: 4);
mu2=Target.Id1.Est2(1,:);
Data_Point2_0 = mvnrnd(mu2',(Sigma2_0+Sigma2_0.')/2, M);

    q1_hat2_0=[mu2(2),mu2(1)];
    covxx2=Sigma2_0(1,1);
    covyy2=Sigma2_0(2,2);
    Sigma2_0(1,1)=covyy2;
    Sigma2_0(2,2)=covxx2;   
    
    SigmaB=zeros(3,3);
    SigmaB(1:2,1:2)=Sigma2_0(1:2,1:2);

elseif N==3
Sigma2_0=P2_save(:,1: 4);
Sigma3_0=P3_save(:,1: 4);

mu2=Target.Id1.Est2(1,:);
mu3=Target.Id1.Est3(1,:);

Data_Point2_0 = mvnrnd(mu2',(Sigma2_0+Sigma2_0.')/2, M);
Data_Point3_0 = mvnrnd(mu3',(Sigma3_0+Sigma3_0.')/2, M);


    q1_hat2_0=[mu2(2),mu2(1)];
    covxx2=Sigma2_0(1,1);
    covyy2=Sigma2_0(2,2);
    Sigma2_0(1,1)=covyy2;
    Sigma2_0(2,2)=covxx2; 
    
    q1_hat3_0=[mu3(2),mu3(1)];
    covxx3=Sigma3_0(1,1);
    covyy3=Sigma3_0(2,2);
    Sigma3_0(1,1)=covyy3;
    Sigma3_0(2,2)=covxx3; 
    
    SigmaB=zeros(3,3);
    SigmaB(1:2,1:2)=Sigma2_0(1:2,1:2);
    SigmaC=zeros(3,3);
    SigmaC(1:2,1:2)=Sigma3_0(1:2,1:2);

end
i=1;
%% Plot trajectories
fig1=figure(1);
set(fig1,'position',[0 0 550 350]);
if N==1
    plot(pd1(:,2),pd1(:,1),'-.','LineWidth',1,'Color',AUV_COL11);
    hold on;
elseif N==2
    plot(pd1(:,2),pd1(:,1),'-.','LineWidth',1,'Color',AUV_COL11);
    hold on;
    plot(pd2(:,2),pd2(:,1),'-.','LineWidth',1,'Color',AUV_COL22);
else
    plot(pd1(:,2),pd1(:,1),'-.','LineWidth',1,'Color',AUV_COL22);
    hold on;
    plot(pd2(:,2),pd2(:,1),'-.','LineWidth',1,'Color',AUV_COL33);
    plot(pd3(:,2),pd3(:,1),'-.','LineWidth',1,'Color',AUV_COL33);

end
    