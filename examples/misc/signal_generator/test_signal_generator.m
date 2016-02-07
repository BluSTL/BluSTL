%% Create the system
nu = 20;
nw = 20;
SG = signal_generator(nu,nw);
SG = init_control(SG);

%% Controller Initialisation
% Time
SG.time = 0:.1:10; % time for the dynamics
SG.ts=1; % sampling time for controller
SG.L=3;  % horizon (# of steps)
SG.nb_stages=1; % repeats time

% Bounds
SG.u_ub(:) = 10;
SG.u_lb(:) = -10;
SG.lambda_rho = 1000;
SG.lambda_t1 = 100;


SG.Wref = kron([sin(SG.time); cos(SG.time)],ones(10,1));


SG.min_rob = 0.01;    
SG.bigM = 1000;

SG.plot_u = [1, 2];
SG.plot_y =[1,2];
SG.plot_w = [1, 2];

%% STL formula
SG.stl_list{1} = '((y1(t)<0) => ev_[0, 5] (y1(t)>5)) and ((y1(t)>5) =>  ev_[0, 5](y1(t)<0)) and ((y2(t)<0) => ev_[0, 5] (y2(t)>5)) and ((y2(t)>5) =>  ev_[0, 5](y2(t)<0))';


%% running stuff

% MONITORING
% for i=1:numel(SG.stl_list)
%     rob = monitor(SG, SG.stl_list{i})
% end

% DETERMINISTIC
fprintf('Computing controller...');
tic;
SG.controller = get_controller(SG,'robust');
toc;
SG = SG.reset_data();
SG = SG.run_deterministic(SG.controller);

% ADVERSARIAL
SG.w_ub(:) = 3;
SG.w_lb(:) = -3;
SG.is_det = 0;

fprintf('Computing controller...');
tic;
SG.controller = get_controller(SG,'robust');
toc;
fprintf('Computing adversary...');
tic;
SG.adversary = get_adversary(SG);
toc;
SG = SG.reset_data();
SG = SG.run_adversarial(SG.controller,SG.adversary);