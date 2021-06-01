%% Video
vid=1;

v = VideoWriter('video_2vehicle_no_rotate');
v.FrameRate=5;
 open(v);

fig1=figure(1);
% fig2=figure(2);
%colors
 
AUV_COL= [0 0 0];
 

AUV_COL33= [0.8 0.8 0]; % yellow
AUV_COL22= [0.1 0.1 0]; % black
AUV_COL11= [0.9 0.2 0]; % red
 
% yaw1=90-yaw1;
% yaw2=90-yaw2;
% yaw3=90-yaw3;
 
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
 set (fig1, 'Units', 'normalized', 'Position', [0,0,1,1]);
 
M=200;
Sigma1_0=P1_save(:,1: 4);
Sigma2_0=P2_save(:,1: 4);

mu1=Target.Id1.Est1(1,:);
mu2=Target.Id1.Est2(1,:);
    

 
Data_Point1_0 = mvnrnd(mu1',(Sigma1_0+Sigma1_0.')/2, M);
Data_Point2_0 = mvnrnd(mu2',(Sigma2_0+Sigma2_0.')/2, M);
    

    q1_hat1_0=[mu1(2),mu1(1)];
    covxx1=Sigma1_0(1,1);
    covyy1=Sigma1_0(2,2);
    Sigma1_0(1,1)=covyy1;
    Sigma1_0(2,2)=covxx1; 
    
    q1_hat2_0=[mu2(2),mu2(1)];
    covxx2=Sigma2_0(1,1);
    covyy2=Sigma2_0(2,2);
    Sigma2_0(1,1)=covyy2;
    Sigma2_0(2,2)=covxx2; 
    
 
    SigmaA=zeros(3,3);
    SigmaA(1:2,1:2)=Sigma1_0(1:2,1:2);
    SigmaB=zeros(3,3);
    SigmaB(1:2,1:2)=Sigma2_0(1:2,1:2);
%     hold off;
N1=1;
N2=5000;
k=1;
i=1; 
while i<length(pd1(:,1))
   
%     if k>20
    hold off;

     figure(fig1);
    % path black
   if i>N2
    plot(pd1(i-N2:i,2),pd1(i-N2:i,1),'-.','LineWidth',1,'Color',AUV_COL11);
    hold on;

    plot(pd2(i-N2:i,2),pd2(i-N2:i,1),'-.','LineWidth',1,'Color',AUV_COL22);
   else
    plot(pd1(1:i,2),pd1(1:i,1),'-.','LineWidth',1,'Color',AUV_COL11);
    hold on;
    plot(pd2(1:i,2),pd2(1:i,1),'-.','LineWidth',1,'Color',AUV_COL22);
   end    

    mu1=Target.Id1.Est1(i,:);
    mu2=Target.Id1.Est2(i,:);
    
    q1_hat1=[mu1(2), mu1(1)];
    q1_hat2=[mu2(2), mu2(1)];
 
    
    
    
    plot(Target_PosEst1(1:i,2),Target_PosEst1(1:i,1),'*','LineWidth',0.2,'Color',AUV_COL11);
    plot(Target_PosEst1(1,2),Target_PosEst1(1,1),'*','MarkerSize',15,'LineWidth',3,'Color',AUV_COL11);

    plot(Target_PosEst2(1:i,2),Target_PosEst2(1:i,1),'*','LineWidth',0.2,'Color',AUV_COL22);
    plot(Target_PosEst2(1,2),Target_PosEst2(1,1),'*','MarkerSize',15,'LineWidth',3,'Color',AUV_COL22);

   

    
    Sigma1=P1_save(:,4*i-3: 4*i);
    Sigma2=P2_save(:,4*i-3: 4*i);
    
    Data_Point1 = mvnrnd(mu1',(Sigma1+Sigma1.')/2, M);
    Data_Point2 = mvnrnd(mu2',(Sigma2+Sigma2.')/2, M);
    
    plot(Data_Point1(:,2),Data_Point1(:,1),'o','Color', AUV_COL11);
    plot(Data_Point2(:,2),Data_Point2(:,1),'o','Color', AUV_COL22);
    
    
%     plot(Data_Point1_0(:,2),Data_Point1_0(:,1),'o','Color', AUV_COL11);
%     plot(Data_Point2_0(:,2),Data_Point2_0(:,1),'o','Color', AUV_COL22);
%     plot(Data_Point3_0(:,2),Data_Point3_0(:,1),'o', 'Color',AUV_COL33);


    covxx1=Sigma1(1,1);
    covyy1=Sigma1(2,2);
    Sigma1(1,1)=covyy1;
    Sigma1(2,2)=covxx1;
    
    
    h1= plot_gaussian_ellipsoid([q1_hat1_0 0], SigmaA,2);
    set(h1,'FaceColor',AUV_COL11); 
    set(h1,'EdgeColor',AUV_COL11); 
    set(h1,'LineStyle','None'); 
    set(h1,'FaceAlpha',0.3);
% 
    h2= plot_gaussian_ellipsoid(q1_hat1, Sigma1(1:2,1:2),2);
    set(h2,'Color',AUV_COL11); 

    covxx2=Sigma2(1,1);
    covyy2=Sigma2(2,2);
    Sigma2(1,1)=covyy2;
    Sigma2(2,2)=covxx2;
    
    h3= plot_gaussian_ellipsoid([q1_hat2_0 0], SigmaB,2);
    set(h3,'FaceColor',AUV_COL22); 
    set(h3,'EdgeColor',AUV_COL22); 
    set(h3,'LineStyle','None'); 
    set(h3,'FaceAlpha',0.3);

    
    h4= plot_gaussian_ellipsoid(q1_hat2, Sigma2(1:2,1:2),2);
    set(h4,'Color',AUV_COL22); 
    
    
    
    plot(Target_Pos(1:i,2),Target_Pos(1:i,1),'-','LineWidth',4,'Color', 'b');
    

%     plot(pd22(1:i,2),pd22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
%     plot(pd33(1:i,2),pd33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
    
if i>N1   
    plot(p1(i-N1:i,2),p1(i-N1:i,1),'.-','LineWidth',2,'Color',AUV_COL11);
    plot(p2(i-N1:i,2),p2(i-N1:i,1),'.-','LineWidth',2,'Color',AUV_COL22);
else
    plot(p1(1:i,2),p1(1:i,1),'.-','LineWidth',2,'Color',AUV_COL11);
    plot(p2(1:i,2),p2(1:i,1),'.-','LineWidth',2,'Color',AUV_COL22);
end
   
%     plot(p22(1:i,2),p22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
%     plot(p33(1:i,2),p33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
%     hold on;
%     view(0,60)    
    % path myellow
   Scale=.5;
    GTF_Simulink_PlotAUV([p1(i,2),p1(i,1),0], [0,0,90-yaw1(i)+180], Scale, 0,AUV_COL11,1);  

    GTF_Simulink_PlotAUV([p2(i,2),p2(i,1),0], [0,0,90-yaw2(i)+180], Scale, 0,AUV_COL22,1);  



%     hold on;
    
    
%         view(0,50)    


    
    
%     GTF_Simulink_PlotAUV([p22(i,2),p22(i,1),0], [0,0,yaw2_22(i)], Scale, 0,AUV_COL22,1);  
%     GTF_Simulink_PlotAUV([p33(i,2),p33(i,1),0], [0,0,yaw3_33(i)], Scale, 0,AUV_COL33,1);  

    Scale=.25*8;
    grid on; %axis equal;
%     axis([-15 15 -5 25 ])
   
    title('Animation of Cooperative Target Tracking and Pursuit')
    xlabel('Y[m]')
    ylabel('X[m]')
%     axis equal;
    if(vid==1)
        drawnow;
        currFrame = getframe(fig1);
        writeVideo(v, currFrame);
    else
        pause(0);
        drawnow;
    end
%     hold off;
    
%     figure(fig2)
%     subplot(3,1,1);hold on;
%     if COM1.Data(i)~=Com1_old
%         stem(COM1.Time(i),1,'Color',[0.8,0.9,0]);
%         title('transmistion signal on vehicle 1');
%         Com1_old=COM1.Data(i);
%     end
%     subplot(3,1,2);hold on;
%     if COM2.Data(i)~=Com2_old
%         stem(COM2.Time(i),1,'k')
%         title('transmistion signal on vehicle 2');
%         Com2_old=COM2.Data(i);
%     end
%     subplot(3,1,3);hold on;
%     if COM3.Data(i)~=Com3_old
%         stem(COM3.Time(i),1,'r')
%         title('transmistion signal on vehicle 3')
%         Com3_old=COM3.Data(i);
%     end 
 i=i+round(k);
 k=k+0.1;
 if k>=10
     k=10;
 end
end

if(vid==1)
    close(v);
%     close(fig);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  Plot data for paper presentation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plot trajectory
fig2=figure(2);
set(fig2,'position',[0 0 550 350]);
%% Vehicle 1


h1=plot(pd1(:,2),pd1(:,1),'--','LineWidth',1,'Color',AUV_COL11);                                    
hold on;
h2=plot(p1(:,2),p1(:,1),'LineWidth',1,'Color',AUV_COL11);

h3=plot(pd2(:,2),pd2(:,1),'--','LineWidth',1,'Color',AUV_COL22);                                    
h4=plot(p2(:,2),p2(:,1),'LineWidth',1,'Color',AUV_COL22);
 


h7=plot(Target_PosEst1(:,2),Target_PosEst1(:,1),'*','MarkerSize',2,'LineWidth',0.2,'Color',AUV_COL11);
h8=plot(Target_PosEst2(:,2),Target_PosEst2(:,1),'*','MarkerSize',2,'LineWidth',0.2,'Color',AUV_COL22);

h10=plot(Target_Pos(:,2),Target_Pos(:,1),'-','LineWidth',2,'Color', 'b');



h20=plot(Target_PosEst1(1,2),Target_PosEst1(1,1),'*','MarkerSize',6,'LineWidth',1.5,'Color',AUV_COL11);
h21=plot(Target_PosEst2(1,2),Target_PosEst2(1,1),'*','MarkerSize',6,'LineWidth',1.5,'Color',AUV_COL22);


h30= plot_gaussian_ellipsoid([q1_hat1_0 0], SigmaA,2);
    set(h30,'FaceColor',AUV_COL11); 
    set(h30,'EdgeColor',AUV_COL11); 
    set(h30,'LineStyle','None'); 
    set(h30,'FaceAlpha',0.2);
 
    
    SigmaA=zeros(3,3);
    SigmaA(1:2,1:2)=Sigma1(1:2,1:2);
h31= plot_gaussian_ellipsoid([q1_hat1 0], SigmaA,2);
    set(h31,'FaceColor',AUV_COL11); 
    set(h31,'EdgeColor',AUV_COL11); 
    set(h31,'LineStyle','None'); 
    set(h31,'FaceAlpha',0.5);
    
    
h32= plot_gaussian_ellipsoid([q1_hat2_0 0], SigmaB,2);
    set(h32,'FaceColor',AUV_COL22); 
    set(h32,'EdgeColor',AUV_COL22); 
    set(h32,'LineStyle','None'); 
    set(h32,'FaceAlpha',0.2);
 
    
    SigmaB=zeros(3,3);
    SigmaB(1:2,1:2)=Sigma2(1:2,1:2);
h33= plot_gaussian_ellipsoid([q1_hat2 0], SigmaB,2);
    set(h33,'FaceColor',AUV_COL22); 
    set(h33,'EdgeColor',AUV_COL22); 
    set(h33,'LineStyle','None'); 
    set(h33,'FaceAlpha',0.5);    
    
    

    

    
%        plot(Data_Point1(:,2),Data_Point1(:,1),'o','Color', AUV_COL11);
grid on; %axis equal;
   
%     title('Animation of Coordinated PF')
    xlabel('Y[m]')
    ylabel('X[m]')
%      axis([-60 120 -60 60 ]);
GTF_Simulink_PlotAUV([p1(end,2),p1(end,1),0], [0,0,90-yaw1(end)+180], Scale*0.5 , 0,AUV_COL11,1);  
GTF_Simulink_PlotAUV([p2(end,2),p2(end,1),0], [0,0,90-yaw2(end)+180], Scale*0.5 , 0,AUV_COL22,1);  


%lgd1=legend([h1 h2 h3 h4 h5 h6],'${\bf p}^{[1]}_{\rm d}$', '${\bf p}^{[1]}$','${\bf p}^{[2]}_{\rm d}$', '${\bf p}^{[2]}$','${\bf p}^{[3]}_{\rm d}$', '${\bf p}^{[3]}$');
lgd1=legend([h2 h4],'${\bf p}^{[1]}$', '${\bf p}^{[2]}$');

set(lgd1,'Interpreter','latex');
lgd1.FontSize=11;

ah1=axes('position',get(gca,'position'),'visible','off');

lgd2=legend(ah1,[h7 h8 h10],'$\hat{\bf q}^{[1]}$','$\hat{\bf q}^{[2]}$','${\bf q}$');
set(lgd2,'Interpreter','latex');
lgd2.FontSize=11;

%% Plot error
L=length(time)-3000;
 
fig3=figure(3);
set(fig3,'position',[0 0 550 200]);
h40=plot(time(1:L),Epursuit1(1:L),'LineWidth',0.5,'Color',AUV_COL11);
hold on;
h41=plot(time(1:L),Epursuit2(1:L),'LineWidth',0.5,'Color',AUV_COL22);
 xlabel('Time[second]');
 ylabel('Errors[m]');


h43=plot(time(1:L),Eest1(1:L),'--','LineWidth',0.5,'Color',AUV_COL11);
h44=plot(time(1:L),Eest2(1:L),'--','LineWidth',0.5,'Color',AUV_COL22);


lgd3=legend([h40 h41],'$||{\bf{e}}^{[1]}||$ ', '$||{\bf{e}}^{[2]}||$');
set(lgd3,'Interpreter','latex');
lgd3.FontSize=11;
ah1=axes('position',get(gca,'position'),'visible','off');
lgd4=legend(ah1, [h43 h44], '$||\tilde{\bf{x}}^{[1]}||$', '$||\tilde{\bf{x}}^{[2]}||$' );
set(lgd4,'Interpreter','latex');
lgd4.FontSize=11;
 
  
