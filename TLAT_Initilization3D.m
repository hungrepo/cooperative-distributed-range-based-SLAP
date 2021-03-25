function [tf,Ts,MPC,Mode,Noise,Tracker,Target,omega_bar]=TLAT_Initilization3D()
tf=200;                         % simulation time
Ts=.1;                           % sampling time
omega_bar=0.1;
%omega_bar=0;
%*** for tracker ***************************************************************
% State Constraint
% vmax=2; vmin=0;
% rmax=0.4; rmin=-0.4;
% 

% 
vmax=10; vmin=-0;
rmax=1; rmin=-1;
qmax=1; qmin=-1;
pmax=1; pmin=-1;


% vmax=20; vmin=-0;
% rmax=2; rmin=-2;
% qmax=2; qmin=-2;
% pmax=2; pmin=-2;

% Input constraint (Accelaration)
CMmax=0.2;CMmin=-0.2;
DMmax=0.05;DMmin=-0.05;

% CMmax=inf;CMmin=-inf;
% DMmax=inf;DMmin=-inf;
% 
% vmax=100; vmin=-100;
% rmax=100; rmin=-100;
% CMmax=2;CMmin=-2;
% DMmax=1;DMmin=-1;

%%      Tracker 1
%[56.5685424949238 -56.5685424949238 19.1341716182545]
Tracker.Id1.State=[55;-60;25;pi;0;0];        % initial position and heading of tracker 
%Tracker.Id1.State=[70; 56;0;0;0;0];        % initial position and heading of tracker 

% Tracker.Id1.State=[50;-35;0;0;0];        % initial position and heading of tracker 
Tracker.Id1.Input=[];                   % torge and force
% tracker 1 constraint
Tracker.Id1.StateMax=[ inf(3,1)];   % upper state constraint
Tracker.Id1.StateMin=[-inf(3,1)];   % lower state constraint
Tracker.Id1.InputMax=[vmax;pmax;qmax;rmax];            % upper input constraint  
Tracker.Id1.InputMin=[vmin;pmin;qmin;rmin];            % lower input constraint
%%      tracker 2
%Tracker.Id2.State=[60; 100;-20;0;0;0];        % initial position and heading of tracker 
%[69.2820323027551 39.9999999999999 -48.2962913144534]
 Tracker.Id2.State=[80;35;-50;-pi/4;0;pi/2];         % initial position and heading of tracker 
Tracker.Id2.Input=[];                   % torge and force
% tracker 2 constraint
Tracker.Id2.StateMax=[ inf(3,1);vmax; rmax];   % upper state constraint
Tracker.Id2.StateMin=[-inf(3,1);vmin; rmin];   % lower state constraint
Tracker.Id2.InputMax=[CMmax;DMmax];            % upper input constraint  
Tracker.Id2.InputMin=[CMmin;DMmin];            % lower input constraint
%%      tracker 3
%Tracker.Id3.State=[-80;100;-40;0;0;0];        % initial position and heading of tracker 
%[-69.2820323027551 75.5578582674797 -4.71274906292428]
Tracker.Id3.State=[-70; 75; -4;0;0;0];         % initial position and heading of tracker 
Tracker.Id3.Input=[];                   % torge and force
% tracker 2 constraint
Tracker.Id3.StateMax=[ inf(3,1);vmax; rmax];   % upper state constraint
Tracker.Id3.StateMin=[-inf(3,1);vmin; rmin];   % lower state constraint
Tracker.Id3.InputMax=[CMmax;DMmax];            % upper input constraint  
Tracker.Id3.InputMin=[CMmin;DMmin];            % lower input constraint
%*** for target ****************************************************************
% Velocity constraint
vtx_max=0.5;vtx_min=-0.5; 
vty_max=0.5;vty_min=-0.5;
%%     Target 1
Target.Id1.Pos=[0;0;-5];         % Initial position of target 
% Target.Id1.Pos=[0;5];         % Initial position of target 
Target.Id1.PosMax=[inf;inf];
Target.Id1.PosMin=[-inf;-inf];
Target.Id1.VelMax=[vtx_max;vty_max];
Target.Id1.VelMin=[vtx_min;vty_min];
%Target.Id1.Depth=5;
Target.Id1.Depth=5;

%%      Target 2
% Target.Id2.Pos=[0;0];             % Initial position of target 
Target.Id2.Pos=[0;0];         % Initial position of target 
Target.Id2.PosMax=[inf;inf];
Target.Id2.PosMin=[-inf;-inf];
Target.Id2.VelMax=[vtx_max;vty_max];
Target.Id2.VelMin=[vtx_min;vty_min];
% Target.Id2.Depth=8;
Target.Id2.Depth=8;

%% Velocity model of target
Target.VelType='sine';
if strcmp(Target.VelType,'constant')
    Target.Id1.Vel=[0.2;0.2;0];
    Target.Id2.Vel=[0.2;0.2];
%     Target.Id1.Vel=[0.2;0];
%     Target.Id2.Vel=[0.2;0];
else
    Target.omega=0.1;
    Target.Id1.Vel=0*[0.2+0.1*cos(Target.omega*Target.Id1.Pos(1,1));0.2+0.1*sin(Target.omega*Target.Id1.Pos(1,1));0];
    Target.Id2.Vel=0*[0.2+0.1*cos(Target.omega*Target.Id2.Pos(1,1));0.2+0.1*sin(Target.omega*Target.Id2.Pos(1,1))];
end

%*** For target estimation 
%%       For tracker 1
Target.Id1.PosHat1=Target.Id1.Pos+2*[-15;6; -10];  % Initial position  
% Target.Id1.PosHat0=Target.Id1.Pos+[-30;-15];
% Target.Id1.PosHat0=Target.Id1.Pos+[-20; 10];  % Initial position
% Target.Id1.PosHat0=[-15;-15];
Target.Id1.VelHat0=Target.Id1.Vel+[.2;.2;-.1];  % Initial velocity   
Target.Id1.P1=10*diag([10 10 10 0.1 0.1 0.01]);
%%       For tracker 2
Target.Id1.PosHat2=Target.Id1.Pos+2*[+10;-6; -8];  % Initial position  
Target.Id1.VelHat0=Target.Id1.Vel+[.2;.2;-.1];  % Initial velocity   
Target.Id1.P2=10*diag([10 10 10 0.1 0.1 0.01]);

%%       For tracker 3
Target.Id1.PosHat3=Target.Id1.Pos+2*[10;10; -8];  % Initial position  
Target.Id1.VelHat0=Target.Id1.Vel+[.2;.2;-.1];  % Initial velocity   
Target.Id1.P3=10*diag([20 10 10 0.1 0.1 0.01]);

%*** Optimal control problem
MPC.Np = 6;                            % number of control intervals
MPC.Tp = Ts*MPC.Np;                     % Time horizon
MPC.rho1 = 0e0;                         % weight for tracking cost  % MPC.rho1 = 1e0 for TTST with terminal set                         
MPC.rho2 = 1;                           % weight for terminal cost
MPC.Terminalset=false;
MPC.R=15;
MPC.Colision=false;
MPC.Rangemodel=false;                    % using range model
MPC.gamma=1e-7;
MPC.epsilon=1e-6;
MPC.Q=0*1e-4*diag([1,1,0,0,0,0,0,0,0,1,1,0]);
MPC.R=0*diag([1,1,1,1]);  
MPC.E=1e-3*diag([1,10,1,10]);
MPC.rho_track=0.00;
MPC.dmax=100;                           % maximum range measurement   
%*** Setup Mode
Mode=2;                                 % Mode=1, if we know true trajectories. Mode=2; if we use estimated from EKF
%*** measurment noise
Tracker.Id1.sigma1=0.5;
Tracker.Id1.sigma2=0.5;
Tracker.Id2.sigma1=0.5;
Tracker.Id2.sigma2=0.5;
rng('default');
Noise.TT11=Tracker.Id1.sigma1*randn(tf/Ts+1,1);
Noise.TT12=Tracker.Id2.sigma1*randn(tf/Ts+1,1);
Noise.TT21=Tracker.Id1.sigma2*randn(tf/Ts+1,1);
Noise.TT22=Tracker.Id2.sigma2*randn(tf/Ts+1,1);

% %*** Collor of tracker and target
% Tracker.Id1.Col= [0 0 0]/255;                           % black
% Tracker.Id2.Col= [0 0 255]/255;                         % Blue 
% Target.Id1.Col= [255 0 0]/255;                          % Red
% Target.Id2.Col= [0,128,128]/255;                        % red 
% Target.Id1.Est_Col= Target.Id1.Col+[-250 0 50]/255;  
% Target.Id2.Est_Col= Target.Id2.Col+[0 50 50]/255;

 
%*** Collor of tracker and target
Tracker.Id1.Col= [0 0 0]/255;                           % black
Tracker.Id2.Col= [0 0 255]/255;                         % Blue 
Target.Id1.Col= [255 0 0]/255;                          % Red
Target.Id2.Col= [0.3 .75,.93];                        % red 
Target.Id1.Est_Col= [0 0.5 0];  
Target.Id2.Est_Col= 'magenta';
