function Rot= Rotation_matrix(phi,theta,psi)
c_phi=cos(phi);
c_theta=cos(theta);
c_psi=cos(psi);
s_phi=sin(phi);
s_theta=sin(theta);
s_psi=sin(psi);
Rot =  [  c_psi*c_theta  -s_psi*c_phi+ c_psi*s_theta*s_phi   s_psi*s_phi+c_psi*c_phi*s_theta;
          s_psi*c_theta   c_psi*c_phi+ s_phi*s_theta*s_psi  -c_psi*s_phi+s_theta*s_psi*c_phi;
         -s_theta         c_theta*s_phi                      c_theta*c_phi];           
end