 %% Video
vid=0;
fig1=figure(1);
fig2=figure(2);
%colors
 
AUV_COL= [0 0 0];
 

AUV_COL33= [0.8 0.8 0]; % yellow
AUV_COL22= [0.1 0.1 0]; % black
AUV_COL11= [0.9 0.2 0]; % red
 
% yaw1=90-yaw1;
 
 
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
 set (fig1, 'Units', 'normalized', 'Position', [0,0,1,1]);
 
M=500;
Sigma1_0=P1_save(:,1: 4);


mu1=Target.Id1.Est1(1,:);
Data_Point1_0 = mvnrnd(mu1',(Sigma1_0+Sigma1_0.')'/2, M);
 


q1_hat1_0=[mu1(2),mu1(1)];
    covxx=Sigma1_0(1,1);
    covyy=Sigma1_0(2,2);
    Sigma1_0(1,1)=covyy;
    Sigma1_0(2,2)=covxx; 
    
%     hold off;

N1=100;
N2=500; 
for i=1:10:size(pd1,1)%data.i,
    hold off
    figure(fig1);
    % path black
   if i>N2
    plot(pd1(i-N2:i,1),pd1(i-N2:i,2),'-.','LineWidth',0.2,'Color',AUV_COL11);
    hold on;
   else
    plot(pd1(1:i,1),pd1(1:i,2),'-.','LineWidth',0.2,'Color',AUV_COL11);
    hold on;   
   end   

    mu1=Target.Id1.Est1(i,:);
    q1_hat1=[mu1(1), mu1(2)];
 
    
    plot(Target_PosEst1(1:i,1),Target_PosEst1(1:i,2),'*','LineWidth',0.2,'Color',AUV_COL11);
 
    
    Sigma1=P1_save(:,4*i-3: 4*i);
 
    
    Data_Point1 = mvnrnd(mu1',(Sigma1+Sigma1.')/2, M);
 
    
%     hold off;
   plot(Data_Point1(:,1),Data_Point1(:,2),'o','Color', AUV_COL11);
 
      
%     plot(Data_Point1_0(:,2),Data_Point1_0(:,1),'o','Color', AUV_COL11);
    
%     covxx1=Sigma1(1,1);
%     covyy1=Sigma1(2,2);
%     Sigma1(1,1)=covyy1;
%     Sigma1(2,2)=covxx1;
%     
     
     
    h1= plot_gaussian_ellipsoid(q1_hat1_0, Sigma1_0(1:2,1:2),2);
    set(h1,'Color',AUV_COL11); 

    h2= plot_gaussian_ellipsoid(q1_hat1, Sigma1(1:2,1:2),2);
    set(h2,'Color',AUV_COL11); 
     

 

%     pause(1);
    
    plot(Target_Pos(1:i,1),Target_Pos(1:i,2),'-','LineWidth',1,'Color', 'b');
    
 
    
   if i>N1
    plot(p1(i-N1:i,1),p1(i-N1:i,2),'.-','LineWidth',0.5,'Color',AUV_COL11);
   else
    plot(p1(1:i,1),p1(1:i,2),'.-','LineWidth',0.5,'Color',AUV_COL11);
   end   
 
    hold on;
    %view(45,20)    
    % path myellow
   Scale=.5;
 
    GTF_Simulink_PlotAUV([p1(i,1),p1(i,2),0], [0,0,yaw1(i)+180], Scale, 0,AUV_COL11,1);  
    
    axis([-60 120 -60 40 ]);
%    axis([-inf inf -inf inf ]);

    

    Scale=.25*8;
    grid on; %axis equal;
   
    title('Animation of Coordinated PF')
    xlabel('Y[m]')
    ylabel('X[m]')
%    axis equal;
    if(vid==1)
        drawnow;
        currFrame = getframe(fig);
        writeVideo(animation, currFrame);
    else
        pause(0);
        drawnow;
    end
%                  axis([-70 50 -60 120 ]);

    hold off;
    
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
end

if(vid==1)
    close(animation);
%     close(fig);
end