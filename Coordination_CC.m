%% This code implements the coordination control algorithm for the cooperative pursuit used in the paper 
%% with continous communication

%   Cooperative distributed estimation and control of multiple autonomous vehicles 
%   for range-based underwater target localization and pursuit

%   Authors: Nguyen T. Hung, Francisco Rego, Antonio Pascoal, Institute for System and Robotic, IST, Lisbon
%   Contact: nguyen.hung@tecnico.ulisboa.pt
%   More information: https://nt-hung.github.io/research/Range-based-target-localization/

%   
%% ======================================== Coordination pursuit controller ===================================================================
function [Tracker1, Tracker2, Tracker3] = Coordination_CC(Tracker1, Tracker2,Tracker3, kc)
    Tracker1.vc(end+1) = -kc*(Tracker1.gamma(end) - Tracker3.gamma(end));                  % Tracker 2 is the in-neighbor of Tracker 1
    Tracker2.vc(end+1) = -kc*(Tracker2.gamma(end) - Tracker1.gamma(end));                  % Tracker 3 is the in-neighbor of Tracker 2
    Tracker3.vc(end+1) = -kc*(Tracker3.gamma(end) - Tracker2.gamma(end));                  % Tracker 1 is the in-neighbor of Tracker 3

    
    