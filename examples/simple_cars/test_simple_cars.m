clear;
close all;
%% Create the system

%A1 = [1, 1; 0, 1 ]; %[x y]_t = [x y]_{t-1} + [1 1; 0 1][x y]_{t-1} + [0; 1]u_{t-1} 
SC = simple_cars();

%% Controller Initialisation
% Time
SC.time = 0:.1:30; % time for the dynamics
SC.ts=.2; % sampling time for controller
SC.L=15;  % horizon (# of steps)
SC.nb_stages=1; % repeats time

% Input constraints
SC.u_lb=-1;
SC.u_ub=1;

% Disturbance signal
w = 0*SC.time;
w(1:5) =-2;
Wref = w;

SC.Wref = Wref;
SC.w_lb(:) = -.5;
SC.w_ub(:) = .5;

%% Initial state
X1 = [-10 0]';
X2 = [10 0]';
X0 = [X1; X2];
SC.x0 = X0;

%% STL formula
SC.stl_list = {'alw_[0, Inf] ( (abs( x1(t) - x3(t) ) < 2) => alw_[0, 2] ( abs(x2(t)) < 0.1 ))'}; 


%% Plotting
%SC.plot_x = 5;
%SC.plot_w = [7];

%% Running stuff
fprintf('Computing controller...\n');
controller = get_controller(SC);

fprintf('Computing adversary...\n');
adversary = get_adversary(SC) 
fprintf('Running...')

SC = run_adversarial(SC, controller, adversary)
fprintf('\ndone.\n');

