%% Create the system
nu = 2;

A = [-1 0 ;
     0 -1];
B = eye(nu);
C = eye(nu);
D = [];

SG= STLC_lti(A,B,C,D); 
SG.x0= [0 ; 0];

%% Controller Initialisation
% Time
SG.time = 0:1:100; % time for the dynamics
SG.ts=1; % sampling time for controller
SG.L=30;  % horizon (# of steps)
SG.nb_stages=1; % repeats time

% Bounds
SG.u_ub(:) = 10;
SG.u_lb(:) = -10;

%% STL formula

SG.stl_list = {'alw_[10,20] (y1(t) > 5 and y2(t)<-2)'};
SG.min_rob = 0.01;    
SG.bigM = 1000;

%% Initial state

%return;
%% running stuff
fprintf('Computing controller...');
tic;
controller = get_controller(SG)
toc;
run_open_loop(SG, controller);
