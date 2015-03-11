addpath env
addpath ../../src
%% Create the system
QuadSys = quad_system();

%% Controller Initialisation
QuadSys.time = 0:1:100; % time for the dynamics
QuadSys.ts=2; % sampling time for controller
QuadSys.L=10;  % horizon (# of steps)
QuadSys.nb_stages=1; % repeats time

QuadSys.max_react_iter=100;
QuadSys.min_rob = 0.1;
QuadSys.lambda_rho = 10;

% Input constraints
QuadSys.u_ub(:)=10;
QuadSys.u_lb(:)= -10;

% Initial state
QuadSys.x0 = [0.1; 0.1; 0.1; zeros(7,1)];

%% STL formula
%            QSys.stl_list{1} = 'alw_[0,Inf] ( abs(x1(t))<.1 and abs(x2(t))<.1 and abs(x3(t)-1)<.1 )';
QuadSys.stl_list{1} = 'alw_[0, Inf] ( ( x3(t)<1  =>  ev_[0, 10] (x3(t)>10)) and ((x3(t)>10)  =>  ev_[0, 10] (x3(t)<2)))';
QuadSys.stl_list{2} = 'alw_[0,Inf] ( x3(t)<11 and x3(t)>0)';

%           QSys.stl_list{3} = 'ev_[0, 10] (  x1(t)>10 ) and ev_[10, 20] (x1(t)<1)';
%           QSys.stl_list{4} = 'alw_[0,Inf] ( x1(t)<11 and x1(t)>0)';

%% Plotting
QuadSys.plot_x = [1:3];

%% Running stuff
fprintf('Computing controller...\n');

tic
QuadSys.controller = get_controller(QuadSys);
toc

run_deterministic(QuadSys, QuadSys.controller);


