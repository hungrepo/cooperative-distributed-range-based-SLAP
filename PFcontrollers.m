function upf=PFcontrollers(x_robot,x_path,upf,controller,vd)
p=x_robot(1:2);
pd=x_path(1:2);
psi=x_robot(3);
v_robot=upf(1);r_robot=upf(2);v_gamma=upf(3);
%% Compute PF error 
if strcmp(controller,'Aguiar')
    pd_gamma=x_path(4:5);
    RB_I=[cos(psi)   -sin(psi);
          sin(psi)    cos(psi)]; 
    e_pf=RB_I'*(p-pd);
    delta=.1;
    Delta_inv= [1    0;
                0  1/delta]; 
    kx=.8; ky=0.3;    
    Kk=[kx 0;
        0  ky];
    ud=Delta_inv*(-Kk*tanh(e_pf-[delta;0])+RB_I'*pd_gamma*vd);    
    upf=[ud;vd];
elseif strcmp(controller,'Lapierre')     
    psid=x_path(3);  
    hg=x_path(6);
    cg=x_path(7);
    RI_F=[cos(psid)      sin(psid);                                         % From {I} to {F}
         -sin(psid)      cos(psid)]; 
    e_f=RI_F*(p-pd);
    s1=e_f(1);
    y1=e_f(2);
    psie=psi-psid; 
    
    k1=.2;k2=1;
    k3=0.1*pi/4;
    k4=1;
    k5=1;
    delta=-k3*tanh(k4*y1);
    ydot=v_robot*sin(psie)-hg*cg*v_gamma*s1;
    delta_dot=-k3*k4*(1-(tanh(k4*y1))^2)*ydot;
    % Controller
    %u_d=hg*vd;
    u_d=.5;
    u_gamma=(v_robot*cos(psie)+k1*s1)/hg;
    if psie==delta
       r_d=delta_dot-k5*y1*v_robot+cg*v_gamma*hg;    
    else
       r_d=delta_dot-k5*y1*v_robot*(sin(psie)-sin(delta))/(psie-delta)-k2*(psie-delta)+cg*v_gamma*hg;
    end
    upf=[u_d;r_d;u_gamma];
end
 
 