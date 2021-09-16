%% This ETC mechanism for cooperative control is motivated by the event triggered communication framwork proposed in the paper 
%  
%   Nguyen T. Hung, Antonio M. Pascoal, “Consensus/synchronization of networked nonlinear multiple agent systems 
%   with event-triggered communications”, International Journal of Control, 2020.
%   See more relevant work at: https://nt-hung.github.io/research/cooperative-control-MAS/

%   
%% ======================================== Coordination pursuit controller ===================================================================
function [Tracker1, Tracker2, Tracker3] = Coordination_ETC(Tracker1, Tracker2,Tracker3, kc,t)
    c0 = 0.1;   c1 = 2;   alpha = .05;
    h = c0 + c1*exp(-alpha*t);                                             % Trigering function for ETC     
    Tracker1.h(end+1) = h; 
    Tracker2.h(end+1) = h;
    Tracker3.h(end+1) = h;
% Compute estimation errors
    Tracker1.gamma_tilde(end+1) = Tracker1.gamma(end) - Tracker1.gamma_hat(end);
    Tracker2.gamma_tilde(end+1) = Tracker2.gamma(end) - Tracker2.gamma_hat(end);
    Tracker3.gamma_tilde(end+1) = Tracker3.gamma(end) - Tracker3.gamma_hat(end);
% - Compare the estimation error with the trigering threshold to decide broadcast or not
   % At Tracker 1
    if  abs(Tracker1.gamma_tilde(end)) >= Tracker1.h(end)
        Tracker1.Com_Control(end+1) = 1;                                    % transmit 
        Tracker1.gamma_hat(end) = Tracker1.gamma(end);                      % reset gamma_hat
    else
        Tracker1.Com_Control(end+1) = 0;                                    % dont broadcast at this interation
    end
   % At Tracker 2
    if  abs(Tracker2.gamma_tilde(end)) >= Tracker2.h(end)
        Tracker2.Com_Control(end+1) = 1;                                    % transmit 
        Tracker2.gamma_hat(end) = Tracker2.gamma(end);                      % reset gamma_hat
    else
        Tracker2.Com_Control(end+1) = 0;                                    % dont broadcast at this interation
    end
   % At Tracker 3
    if  abs(Tracker3.gamma_tilde(end)) >= Tracker3.h(end)
        Tracker3.Com_Control(end+1) = 1;                                    % transmit 
        Tracker3.gamma_hat(end) = Tracker3.gamma(end);                      % reset gamma_hat
    else
        Tracker3.Com_Control(end+1) = 0;                                    % dont broadcast at this interation
    end
% Compute correction speeds    
    Tracker1.vc(end+1) = -kc*(Tracker1.gamma(end) - Tracker3.gamma_hat(end));                  % Tracker 2 is the in-neighbor of Tracker 1
    Tracker2.vc(end+1) = -kc*(Tracker2.gamma(end) - Tracker1.gamma_hat(end));                  % Tracker 3 is the in-neighbor of Tracker 2
    Tracker3.vc(end+1) = -kc*(Tracker3.gamma(end) - Tracker2.gamma_hat(end));                  % Tracker 1 is the in-neighbor of Tracker 3

end





    