%% Create the system
GridSys = grid_system();

%% Controller Initialisation
% Time
GridSys.time = 0:1:100; % time for the dynamics
GridSys.ts=1; % sampling time for controller
GridSys.L=15;  % horizon (# of steps)
GridSys.nb_stages=1; % repeats time

% Disturbance signal
w = 0*ones(GridSys.nw,numel(GridSys.time)); 
w(1,10:20)=-0.3;
w(1,21:40)=0.2;
w(1,41:75)=-0.2;
GridSys.Wref = w;

% Bounds
Uu_anc  = 0.6;
GridSys.u_ub = Uu_anc;
GridSys.u_lb = -Uu_anc;
GridSys.u_delta = .2;

%% STL formula

%stl = {'alw_[0, Inf] (not (alw_[0,10] abs(var.Y(1,t))> .1))'};
stl = {'alw_[0, Inf] (abs(var.Y(1,t))< .2)'};
GridSys.stl_list = stl;

GridSys.rob = 0.01;    
GridSys.bigM = 1000;

GridSys.plot_x = [6 13];

%% running stuff
fprintf('Computing controller...');
controller = get_controller(GridSys);
fprintf('\nRunning...');
GridSys = run_deterministic(GridSys, controller);
fprintf('\ndone.\n');







