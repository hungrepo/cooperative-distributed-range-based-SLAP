t=0:0.1:300;
for i=1:length(t)
    qx(i) =  0.1*t(i); 
    qy(i) = 0.05*t(i);
    omega(i) = 0.02+0.0001*t(i); 
 %   gamma = gamma +qx;
    x1(i)=10*sin(omega(i)*t(i)) + qx(i);
    y1(i)=10*cos(omega(i)*t(i)) + qy(i);
% 
%     x = [qx(i) x1(i)];
%     y = [qy(i) y1(i)];
%     annotation('textarrow',x,y,'String',' ')
    
    
%     x = .5*t - 10*sin(.1*t) ;        
%     y = 0 - 10*cos(.1*t);
%     z= 10*sin(0.1*t);
end
plot(x1,y1);
hold on;
plot(qx,qy);
% plot(x2,y2);
% plot(x3,y3);

axis equal;

% figure;
% plot3(x,y,z);