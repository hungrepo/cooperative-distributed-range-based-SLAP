function x_path=paths(pathtype,pathpar,gamma,psid,t)
        if strcmp(pathtype,'cycloid');
           x_path=cycloidpath(gamma,pathpar,psid,t); 
        end
        if strcmp(pathtype,'circle');
           x_path=circlepath(gamma,pathpar,psid,t); 
        end
        if strcmp(pathtype,'sin');
           x_path=sinpath(gamma,pathpar,psid,t); 
        end
        if strcmp(pathtype,'polynominal');
           x_path=polypath(gamma,pathpar,psid,t); 
        end
        if strcmp(pathtype,'Bernoulli');
           x_path=Bernoulli(gamma,pathpar,psid,t); 
        end
        if strcmp(pathtype,'Lawnmover');
           x_path=Lawnmover(gamma,pathpar,psid,t); 
        end
        if strcmp(pathtype,'heart');
           x_path=heart(gamma,pathpar,psid,t); 
        end
end
%% ************************************************************************
function x_path=cycloidpath(gamma,pathpar,psid,t)
    Rx=pathpar.Rx;
    Ry=pathpar.Ry;
    omega=pathpar.omega;
    vx=pathpar.vx;
    vy=pathpar.vy;
    x0=pathpar.x0;
    y0=pathpar.y0;
    pd=[Rx*sin(omega*gamma)+vx*gamma+x0;
        Ry*cos(omega*gamma)+vy*gamma+y0];

    pd_gamma=[Rx*omega*cos(omega*gamma)+vx;
             -Ry*omega*sin(omega*gamma)+vy];
    hg=norm(pd_gamma);
    cg=Rx*omega^2*sin(omega*gamma)*(vy + Ry*omega*cos(omega*gamma)) - Ry*omega^2*sin(omega*gamma)*(vx + Rx*omega*cos(omega*gamma));
    if t==0
        psid0=atan2(pd_gamma(2),pd_gamma(1));
    end
    if t==0
        x_path=[pd;psid0;pd_gamma;hg;cg;gamma];
    else
        x_path=[pd;psid;pd_gamma;hg;cg;gamma];
    end
end
%% ************************************************************************
function x_path=sinpath(gamma,pathpar,psid,t)
% We consider sin path with following parameterization
%       x=a*sin(omega*gamma+phi)+d;
%       y=gamma;
    a=pathpar(1);omega=pathpar(2);phi=pathpar(3);d=pathpar(4);                                            % parameters of sin path
    pd=[a*sin(omega*gamma+phi)+d;gamma];
    pd_gamma=[a*omega*cos(omega*gamma+phi);1];
    hg=sqrt(1+a^2*omega^2*cos(omega*gamma+phi)^2);
    cg=a*omega^2*sin(omega*gamma+phi)/(hg^3);
    if t==0
        psid0=atan2(1,a*omega*cos(omega*gamma+phi));
    end
    if t==0
        x_path=[pd;psid0;pd_gamma;hg;cg;gamma];
    else
        x_path=[pd;psid;pd_gamma;hg;cg;gamma];
    end
end
%% ************************************************************************
function x_path=circlepath(gamma,pathpar,psid,t)
% We consider circle path with following parameterization
%       x=x0+a*cos(gamma);
%       y=y0+a*sin(gamma);
    a=pathpar(1);x0=pathpar(2);y0=pathpar(3);                                            % parameters of sin path
    pd=[x0+a*cos(gamma);y0+a*sin(gamma)];
    pd_gamma=[-a*sin(gamma);a*cos(gamma)];
    hg=a;
    cg=1/a;
    if t==0
        psid0=atan2(a*cos(gamma),-a*sin(gamma));
    end
    if t==0
        x_path=[pd;psid0;pd_gamma;hg;cg;gamma];
    else
        x_path=[pd;psid;pd_gamma;hg;cg;gamma];
    end
end
%% ************************************************************************
function x_path=polypath(gamma,pathpar,psid,t)
% We consider polynominal path (order 5) with following parameterization
%       a=[a1;a2;a3;a4;a5;a6]; b=[b1;b2;b3;b4;b5;b6];
%       phi=[(gamma+c)^5;(gamma+c)^4;(gamma+c)^3;(gamma+c)^2;(gamma+c);1]
%       x=a'*phi;
%       y=b'*phi;
    % Path from yogang
%         a=[0;0;0;0;1;0];
%         b=[0;0;-3.255e-04; 0.263;-72.550; 6.505e+03]; % parameters of polynominal path
%         c=238;
    % Path from Bahareh
        a=pathpar(:,1);b=pathpar(:,2);
%         a=[-25.0647738972264;70.4704087877383;-76.2420352989336;32.7607709906985;7.04154149110126; 1];
%         b=[-25.8120636399255;60.5162465481219;-26.4811860051878;-16.2546057813239;7.05800506702106; 1]; % parameters of polynominal path
        c=0;
%     
    z=gamma+c;
    phi=[z^5;z^4;z^3;z^2;z;1];
    dphi=[5*z^4;4*z^3;3*z^2;2*z;1];
    ddphi=[20*z^3;12*z^2;6*z;2];
    pd=[a'*phi;b'*phi];
    xdot_gamma=a(1:5)'*dphi;     ydot_gamma=b(1:5)'*dphi;
    xddot_gamma=a(1:4)'*ddphi;   yddot_gamma=b(1:4)'*ddphi;
    pd_gamma=[xdot_gamma;ydot_gamma];
    hg=sqrt(xdot_gamma^2+ydot_gamma^2);
    cg=(xdot_gamma*yddot_gamma-ydot_gamma*xddot_gamma)/(hg^3);
    if t==0
        psid0=atan2(ydot_gamma,xdot_gamma);
    end
    if t==0
        x_path=[pd;psid0;pd_gamma;hg;cg;gamma];
    else
        x_path=[pd;psid;pd_gamma;hg;cg;gamma];
    end
end
%% ************************************************************************
function x_path=Bernoulli(gamma,pathpar,psid,t)
% x=a*cos(t)/(1+(sin(t))^2);
% y=a*sin(t)*cos(t)/(1+sin(t)^2);
    a=pathpar;                                            % parameters of Bernoulli path
    z=1+sin(gamma)^2;
    xd=a*cos(gamma)/z;
    yd=a*sin(gamma)*cos(gamma)/z;
    xdot_gamma =(a*sin(gamma)*(sin(gamma)^2 - 3))/z^2;
%     ydot_gamma =-a*2*sin(gamma)^2/z^2;
    ydot_gamma=-(a*(3*sin(gamma)^2 - 1))/(sin(gamma)^2 + 1)^2;
    pd=[xd;yd];
    pd_gamma=[xdot_gamma;ydot_gamma];
    hg=sqrt(xdot_gamma^2+ydot_gamma^2);
    num=3*sqrt(2)*cos(gamma);
    den=a*sqrt(3-cos(2*gamma));
    cg=num/den;
    if t==0
        psid0=atan2(ydot_gamma,xdot_gamma);
    end
    if t==0
        x_path=[pd;psid0;pd_gamma;hg;cg;gamma];
    else
        x_path=[pd;psid;pd_gamma;hg;cg;gamma];
    end
end
%% ************************************************************************
function x_path=Lawnmover(gamma,pathpar,psid,t)
a=pathpar(1);R1=pathpar(2);R2=pathpar(3);d=pathpar(4);
if gamma<=1
    xd=a*gamma; yd=d;
    pd=[xd;yd];
    pd_gamma=[a;0];
    hg=a;
    cg=0;
elseif (1<gamma)&&(gamma<=2)
    z=gamma-1;
    xd=a+(R1-d)*sin(z*pi); yd=d+(R1-d)*(1-cos(z*pi));
    pd=[xd;yd];
    pd_gamma=[(R1-d)*pi*cos(z*pi);(R1-d)*pi*sin(z*pi)];
    hg=(R1-d)*pi;
    cg=1/(R1-d); 
elseif (2<gamma)&&(gamma<=3)                         % (2<gamma)&&(gamma<=3)
    z=gamma-2;
    xd=a*(1-z); yd=2*R1-d;
    pd=[xd;yd];
    pd_gamma=[-a;0];
    hg=a;
    cg=0;
elseif (3<gamma)&&(gamma<=4)
    z=gamma-3;
    xd=0-(R2+d)*sin(z*pi); yd=2*R1-d+(R2+d)*(1-cos(z*pi));
    pd=[xd;yd];
    pd_gamma=[-(R2+d)*pi*cos(z*pi);(R2+d)*pi*sin(z*pi)];
    hg=R2*pi;
    cg=-1/R2;    
else
    z=gamma-4;
    xd=a*z; yd=2*(R1+R2)+d;
    pd=[xd;yd];
    pd_gamma=[a;0];
    hg=a;
    cg=0;
end

    if t==0
        psid0=atan2(pd_gamma(2),pd_gamma(1));
    end
    if t==0
        x_path=[pd;psid0;pd_gamma;hg;cg;gamma];
    else
        x_path=[pd;psid;pd_gamma;hg;cg;gamma];
    end
end
%% ************************************************************************
 function x_path=heart(gamma,psid,t)
% x=a*cos(t)/(1+(sin(t))^2);
% y=a*sin(t)*cos(t)/(1+sin(t)^2);
    z=gamma;                                            % parameters of Bernoulli path
    xd=16*sin(z)^3;
    yd=13*cos(z)-5*cos(2*z)-2*cos(3*z)-cos(4*z);
    xdot_gamma =24*sin(2*z)*sin(z);
    xddot_gamma=24*(2*cos(2*z)*sin(z)+sin(2*z)*cos(z));
    ydot_gamma =-13*sin(z)+10*sin(2*z)+6*sin(3*z)+4*sin(4*z);
    yddot_gamma=-13*cos(z)+20*cos(2*z)+18*cos(3*z)+16*cos(4*z);
    pd=[xd;yd];
    pd_gamma=[xdot_gamma;ydot_gamma];
    hg=sqrt(xdot_gamma^2+ydot_gamma^2);
    cg=(xdot_gamma*yddot_gamma-ydot_gamma*xddot_gamma)/(hg^3);
    if t==0
        psid0=atan2(ydot_gamma,xdot_gamma);
    end
    if t==0
        x_path=[pd;psid0;pd_gamma;hg;cg;gamma];
    else
        x_path=[pd;psid;pd_gamma;hg;cg;gamma];
    end
end
