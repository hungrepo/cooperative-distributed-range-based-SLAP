 %% Video
vid=1;

v = VideoWriter('video_1vehicle');
v.FrameRate=5;
open(v);
fig1=figure(1);

 
AUV_COL= [0 0 0];
 

AUV_COL33= [0.8 0.8 0]; % yellow
AUV_COL22= [0.1 0.1 0]; % black
AUV_COL11= [0.9 0.2 0]; % red
 

yaw1 = psi*180/pi;


% yaw1=90-yaw1;
 
 
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
 set (fig1, 'Units', 'normalized', 'Position', [0,0,1,1]);
 
M=50;
Sigma1_0=P_save(:,1: 4);


mu1=x_hat(1,:);
Data_Point1_0 = mvnrnd(mu1',(Sigma1_0+Sigma1_0.')'/2, M);
 


q1_hat1_0=[mu1(2),mu1(1)];
    covxx=Sigma1_0(1,1);
    covyy=Sigma1_0(2,2);
    Sigma1_0(1,1)=covyy;
    Sigma1_0(2,2)=covxx; 
    
    
    SigmaA=zeros(3,3);
    SigmaA(1:2,1:2)=Sigma1_0(1:2,1:2);
%     hold off;

N1=5000;
N2=5000; 
k=100;
i=1;
while i<length(pd(:,1))
    hold off
    figure(fig1);
    % path black
   if i>N2
    plot(pd(i-N2:i,2),pd(i-N2:i,1),'-.','LineWidth',0.2,'Color',AUV_COL11);
    hold on;
   else
    plot(pd(1:i,2),pd(1:i,1),'-.','LineWidth',0.2,'Color',AUV_COL11);
    hold on;   
   end   

    mu1=x_hat(i,:);
    q1_hat1=[mu1(2), mu1(1)];
 
    
    plot(x_hat(1:i,2),x_hat(1:i,1),'*','LineWidth',0.2,'Color',AUV_COL11);
    plot(x_hat(1,2),x_hat(1,1),'*','MarkerSize',15,'LineWidth',3,'Color',AUV_COL11);

 
    
    Sigma1=P_save(:,4*i-3: 4*i);
 
    
    Data_Point1 = mvnrnd(mu1',(Sigma1+Sigma1.')/2, M);
 
    
%     hold off;
   plot(Data_Point1(:,2),Data_Point1(:,1),'o','Color', AUV_COL11);
 
      
%     plot(Data_Point1_0(:,2),Data_Point1_0(:,1),'o','Color', AUV_COL11);
    
    covxx1=Sigma1(1,1);
    covyy1=Sigma1(2,2);
    Sigma1(1,1)=covyy1;
    Sigma1(2,2)=covxx1;
    
    if (rem(i-1,20)==0)
        h10 = plot([p(i,2) q(i,2)],[p(i,1) q(i,1)],'m-.','LineWidth',2)  ; 
    end 
     
    h1= plot_gaussian_ellipsoid([q1_hat1_0 0], SigmaA,2);
    set(h1,'FaceColor',AUV_COL11); 
    set(h1,'EdgeColor',AUV_COL11); 
    set(h1,'LineStyle','None'); 
    set(h1,'FaceAlpha',0.3);
    
    

    h2= plot_gaussian_ellipsoid(q1_hat1, Sigma1(1:2,1:2),2);
    set(h2,'Color',AUV_COL11); 
     

 

%     pause(1);
    
%    plot(q(1:i,2),q(1:i,1),'-','LineWidth',4,'Color', 'b');
    
    plot(q(1:i,2),q(1:i,1),'-','LineWidth',4,'Color', 'b');
    plot(q(i,2),q(i,1),'*','MarkerSize',20, 'LineWidth', 3, 'Color', 'b');
    
    
   if i>N1
    plot(p(i-N1:i,2),p(i-N1:i,1),'.-','LineWidth',0.5,'Color',AUV_COL11);
   else
    plot(p(1:i,2),p(1:i,1),'.-','LineWidth',0.5,'Color',AUV_COL11);
   end   
 
    hold on;
    %view(45,20)    
    % path myellow
   Scale=.5;
 
    GTF_Simulink_PlotAUV([p(i,2),p(i,1),0], [0,0,90-yaw1(i)+180], Scale, 0,AUV_COL11,1);  
    
if (i<=500)
    axis([-60 40 -60 30 ]);
end
%    axis([-inf inf -inf inf ]);

    

    Scale=.25*8;
    grid on; %axis equal;
   
%     title('Animation of Coordinated PF')
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
% i = i+50;

%   i=i+round(k);
%   k=k+0.1;
%   if k>=100
%       k=100;
%   end
i = i+5;

if i>=100
    i=i+10;
    
end
 end

if(vid==1)
    close(v);
%     close(fig);
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%  Plot data for paper presentation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot trajectory
fig2=figure(2);
set(fig2,'position',[0 0 550 350]);
hold on;
h1=plot(pd(:,2),pd(:,1),'--','LineWidth',1,'Color',AUV_COL11);                                    
hold on;
h2=plot(p(:,2),p(:,1),'LineWidth',1,'Color',AUV_COL11);


h3=plot(x_hat(:,2),x_hat(:,1),'*','MarkerSize',2,'LineWidth',0.2,'Color',AUV_COL11);
h4=plot(q(:,2),q(:,1),'-','LineWidth',2,'Color', 'b');

h5=plot(x_hat(1,2),x_hat(1,1),'*','MarkerSize',6,'LineWidth',1.5,'Color',AUV_COL11);
h6= plot_gaussian_ellipsoid([q1_hat1_0 0], SigmaA,2);
    set(h6,'FaceColor',AUV_COL11); 
    set(h6,'EdgeColor',AUV_COL11); 
    set(h6,'LineStyle','None'); 
    set(h6,'FaceAlpha',0.2);
 
    
    SigmaA=zeros(3,3);
    SigmaA(1:2,1:2)=Sigma1(1:2,1:2);
h7= plot_gaussian_ellipsoid([q1_hat1 0], SigmaA,2);
    set(h7,'FaceColor',AUV_COL11); 
    set(h7,'EdgeColor',AUV_COL11); 
    set(h7,'LineStyle','None'); 
    set(h7,'FaceAlpha',0.5);
%        plot(Data_Point1(:,2),Data_Point1(:,1),'o','Color', AUV_COL11);
grid on; %axis equal;
   
%     title('Animation of Coordinated PF')
    xlabel('Y[m]')
    ylabel('X[m]')
%      axis([-60 120 -60 60 ]);
GTF_Simulink_PlotAUV([p(end,2),p(end,1),0], [0,0,90-yaw1(end)+180], Scale*0.5 , 0,AUV_COL11,1);  
lgd1=legend([h1 h2],'${\bf p}_{\rm d}$', '${\bf p}$');
set(lgd1,'FontSize',12,'Interpreter','latex');
%lgd1.FontSize=11;
 ah1=axes('position',get(gca,'position'),'visible','off');
%lgd2=legend(ah1,[h3 h4 h5],'${\bf q}$','$\hat{\bf q}$','$\hat{\bf q}(0)$');
lgd2=legend(ah1,[h3 h4],'$\hat{\bf q}$','${\bf q}$');

set(lgd2,'FontSize',12,'Interpreter','latex');
%lgd2.FontSize=11;

%% Plot pursuit error 
L=length(time)-2500;
 
fig3=figure(3);
set(fig3,'position',[0 0 550 200]);
h40=plot(time(1:L),E_Pursuit(1:L),'LineWidth',1,'Color',AUV_COL11);
hold on;
h41=plot(time(1:L),E_Loc(1:L),'--','LineWidth',1,'Color',AUV_COL11);
lgd3=legend([h40 h41],'$||{\bf{e}}||$ ', '$||\tilde{\bf{x}}||$ ');
set(lgd3,'Interpreter','latex');
lgd3.FontSize=11;
    xlabel('$t(s)$','FontSize',12,'Interpreter','latex')
    ylabel('Errors[m]','FontSize',12,'Interpreter','latex')