addpath ../../src

%% Defines the plant
A = [0 1 ;
     0 0];
Bu = [0;1];
Bw = [0;0]; 

C = [1 0];
Du = 0;
Dw = 1;

Sys= STLC_lti(A,Bu,Bw,C,Du,Dw); 

%% Controller Initialisation
% Time
Sys.time = 0:1:100; % time for the dynamics
Sys.ts=1; % sampling time for controller
Sys.L=10;  % horizon (# of steps)
Sys.nb_stages=1; % repeats time

% Constraints  on inputs
Sys.u_ub(:) = 10;
Sys.u_lb(:) = -10;

%% STL formula

Sys.stl_list = {'alw (ev_[10,20] ( y1(t) < 0.1 and y1(t)>-0.1 ))'};
Sys.min_rob = 0.01;    

%% Initial state
Sys.x0= [1 ; 1];

%% Creating the controller
controller = get_controller(Sys)

%% Running in open loop mode
Sys = run_open_loop(Sys, controller);

%% Running in closed loop 

%run_deterministic(Sys, controller);