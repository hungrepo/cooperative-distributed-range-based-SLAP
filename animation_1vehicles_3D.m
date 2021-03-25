 %% Video
vid=1;

v = VideoWriter('video_1vehicle_3D');
v.FrameRate=5;
open(v);
fig1=figure(1);
% fig2=figure(2);
%colors
 
AUV_COL= [0 0 0];
 

AUV_COL33= [0.8 0.8 0]; % yellow
AUV_COL22= [0.1 0.1 0]; % black
AUV_COL11= [0.9 0.2 0]; % red
 
 
 
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
 set (fig1, 'Units', 'normalized', 'Position', [0,0,1,1]);
 
M=500;
Sigma1_0=P_save(:,1:6);


mu1=x_hat(1,:);
Data_Point1_0 = mvnrnd(mu1',(Sigma1_0+Sigma1_0.')'/2, M);
 


q1_hat1_0=[mu1(1),mu1(2) mu1(3)];
%     covxx=Sigma1_0(1,1);
%     covyy=Sigma1_0(2,2);
%     Sigma1_0(1,1)=covyy;
%     Sigma1_0(2,2)=covxx; 
    
%     hold off;

N1=500;
N2=5000; 
i=1;
k=100;
while i<length(pd(:,1))
    %data.i,
      hold off
    figure(fig1);
    % path black
   if i>N2
    plot3(pd(i-N2:i,1),pd(i-N2:i,2),pd(i-N2:i,3),'-.','LineWidth',0.2,'Color',AUV_COL11);
    hold on;
   else
    plot3(pd(1:i,1),pd(1:i,2),pd(1:i,3),'-.','LineWidth',0.2,'Color',AUV_COL11);
    hold on;   
   end   

    mu1=x_hat(i,:);
    q1_hat1=[mu1(1), mu1(2) mu1(3)];
 
    
    plot3(x_hat(1:i,1),x_hat(1:i,2),x_hat(1:i,3),'*','LineWidth',0.2,'Color',AUV_COL11);
 
    
   Sigma1=P_save(:,6*i-5: 6*i);
 
    
%    Data_Point1 = mvnrnd(mu1',(Sigma1+Sigma1.')/2, M);
 
    
%     hold off;
%   plot(Data_Point1(:,2),Data_Point1(:,1),'o','Color', AUV_COL11);
 
      
%     plot(Data_Point1_0(:,2),Data_Point1_0(:,1),'o','Color', AUV_COL11);
    
%    covxx1=Sigma1(1,1);
%    covyy1=Sigma1(2,2);
%    Sigma1(1,1)=covyy1;
%    Sigma1(2,2)=covxx1;
    
     
     
    h1= plot_gaussian_ellipsoid(q1_hat1_0, Sigma1_0(1:3,1:3),2);
    set(h1,'FaceColor',AUV_COL11); 
    set(h1,'EdgeColor',AUV_COL11); 
    set(h1,'LineStyle','None'); 
    set(h1,'FaceAlpha',0.2);
%     set(h1,'Color',AUV_COL11); 
% 
%     set(h1,'FaceColor',AUV_COL11); 
%     set(h1,'EdgeColor',AUV_COL11); 
%     set(h1,'LineStyle','-.'); 
%     set(h1,'FaceAlpha',0.3);

    h2= plot_gaussian_ellipsoid(q1_hat1, Sigma1(1:3,1:3),2);
    set(h2,'FaceColor',AUV_COL11); 
    set(h2,'EdgeColor',AUV_COL11); 
    set(h2,'LineStyle','None'); 
    set(h2,'FaceAlpha',0.2);
    
    
%     set(h2,'facealpha',0.1);

%     set(h2,'Color',AUV_COL11); 
     

 

%     pause(1);
    
    plot3(q(1:i,1),q(1:i,2),q(1:i,3),'-','LineWidth',4,'Color', 'b');
        
   if i>N1
    plot3(p(i-N1:i,1),p(i-N1:i,2),p(i-N1:i,3),'.-','LineWidth',0.5,'Color',AUV_COL11);
   else
    plot3(p(1:i,1),p(1:i,2),p(1:i,3),'.-','LineWidth',0.5,'Color',AUV_COL11);
   end   
   hold on;
   % view(i/40,i/80)   ;
    view(201,17);
    % path myellow
   Scale=.5;
%  GTF_Simulink_PlotAUV([Tracker.State(i,1),Tracker.State(i,2),Tracker.State(i,3)], [Tracker.State(i,4)*180/pi+180,Tracker.State(i,5)*180/pi,(Tracker.State(i,6))*180/pi+180], Scale*1.2, 0, AUV_COL1,1); 
GTF_Simulink_PlotAUV([Tracker.State(i,1),Tracker.State(i,2),Tracker.State(i,3)], [Tracker.State(i,4)*180/pi+180,Tracker.State(i,5)*180/pi,(Tracker.State(i,6))*180/pi+180], Scale*1.2, 0, AUV_COL11,1); 

%     GTF_Simulink_PlotAUV([p1(i,1),p1(i,2),p1(i,3)], [Tracker.State(i,4)*180/pi+180,Tracker.State(i,5)*180/pi,Tracker.State(i,6)*180/pi+180], Scale, 0,AUV_COL11,1);  
    
%     axis([-60 120 -60 40 ]);
%    axis([-inf inf -inf inf ]);
    plot3(q(i,1),q(i,2),q(i,3),'*','MarkerSize',20, 'LineWidth', 3, 'Color', 'b');

        
    if (rem(i-1,20)==0)
        tem=[[p(i,:)'],[q(i,:)']];
            h4=plot3(tem(1,:),tem(2,:),tem(3,:),'m-.','LineWidth',2)  ; 
        
 %       h10 = plot3([p(i,2) q(i,2)],[p(i,1) q(i,1)],'m-.','LineWidth',2)  ; 
    end 

    Scale=.25*8;
    grid on; %axis equal;
   
    title('Animation of range-based target localization with a single tracker')
    xlabel('Y[m]')
    ylabel('X[m]')
%    axis equal;
    if(vid==1)
        drawnow;
        currFrame = getframe(fig1);
        writeVideo(v, currFrame);
    else
        pause(0);
        drawnow;
    end
%                  axis([-70 50 -60 120 ]);

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
i = i+5;

% if i>=100
%     i=i+2;
%     
% end
end

if(vid==1)
    close(v);
%     close(fig);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  Plot data for paper presentation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fig2=figure(2);
set(fig2,'position',[0 0 550 350]);
hold on;
h1=plot3(pd(:,1),pd(:,2),pd(:,3),'--','LineWidth',1,'Color',AUV_COL11);                                    
hold on;
h2=plot3(p1(:,1),p1(:,2),p1(:,3),'LineWidth',1,'Color',AUV_COL11);


h3=plot3(x_hat(:,1),x_hat(:,2),x_hat(:,3),'*','MarkerSize',2,'LineWidth',0.2,'Color',AUV_COL11);
h4=plot3(q(:,1),q(:,2),q(:,3),'-','LineWidth',2,'Color', 'b');

h5=plot3(x_hat(1,1),x_hat(1,2),x_hat(1,3),'*','MarkerSize',6,'LineWidth',1.5,'Color',AUV_COL11);
h6= plot_gaussian_ellipsoid(q1_hat1_0, Sigma1_0(1:3,1:3),2);
    set(h6,'FaceColor',AUV_COL11); 
    set(h6,'EdgeColor',AUV_COL11); 
    set(h6,'LineStyle','None'); 
    set(h6,'FaceAlpha',0.2);
 
    
h7= plot_gaussian_ellipsoid(q1_hat1, Sigma1(1:3,1:3),2);
    set(h7,'FaceColor',AUV_COL11); 
    set(h7,'EdgeColor',AUV_COL11); 
    set(h7,'LineStyle','None'); 
    set(h7,'FaceAlpha',0.5);
%        plot(Data_Point1(:,2),Data_Point1(:,1),'o','Color', AUV_COL11);
grid on; %axis equal;
   
%     title('Animation of Coordinated PF')
%     xlabel('Y[m]')
%     ylabel('X[m]')
%      axis([-60 120 -60 60 ]);
GTF_Simulink_PlotAUV([Tracker.State(end,1),Tracker.State(end,2),Tracker.State(end,3)], [Tracker.State(end,4)*180/pi+180,Tracker.State(end,5)*180/pi,(Tracker.State(end,6))*180/pi+180], Scale, 0, AUV_COL11,1); 
lgd1=legend([h1 h2],'${\bf p}_{\rm d}$', '${\bf p}$');
set(lgd1,'FontSize',12,'Interpreter','latex');
%lgd1.FontSize=11;
 ah1=axes('position',get(gca,'position'),'visible','off');
%lgd2=legend(ah1,[h3 h4 h5],'${\bf q}$','$\hat{\bf q}$','$\hat{\bf q}(0)$');
lgd2=legend(ah1,[h3 h4],'$\hat{\bf q}$','${\bf q}$');

set(lgd2,'FontSize',12,'Interpreter','latex');
%lgd2.FontSize=11;

%% Plot pursuit error 
L=length(time)-0;
 
fig3=figure(3);
set(fig3,'position',[0 0 550 200]);
h40=plot(time(1:L),Epursuit1(1:L),'LineWidth',0.5,'Color',AUV_COL11);
hold on;
h41=plot(time(1:L),Eest1(1:L),'--','LineWidth',0.5,'Color',AUV_COL11);
lgd3=legend([h40 h41],'$||{\bf{e}}||$ ', '$||\tilde{\bf{x}}||$ ');
set(lgd3,'FontSize',12,'Interpreter','latex');
%lgd3.FontSize=11;
    xlabel('$t(s)$','FontSize',12,'Interpreter','latex')
    ylabel('Errors [m]','FontSize',12,'Interpreter','latex')
