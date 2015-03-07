addpath env

%% Create the system
QuadSys = quad_system();

%% Controller Initialisation
% Time
QuadSys.time = 0:1:100; % time for the dynamics
QuadSys.ts=0.5; % sampling time for controller
QuadSys.L=10;  % horizon (# of steps)
QuadSys.nb_stages=1; % repeats time

%% Disturbance signal
w = 0*ones(1,numel(QuadSys.time));
Wref = w;
QuadSys.Wref = w;

% Bounds
QuadSys.u_ub = 10;
QuadSys.u_lb = -10;
QuadSys.u_delta = 20;

%% STL formula

QuadSys = getSpecs(QuadSys,3,3);

QuadSys.rob = 0.01;    
QuadSys.bigM = 1000;

%% Initial state
QuadSys.x0 = [2; 2; 0.0; zeros(7,1)];

return;
%% running stuff
fprintf('Computing controller...');
controller = get_controller(QuadSys);
fprintf('\nRunning...');
run_deterministic(QuadSys, controller);
fprintf('\ndone.\n');







