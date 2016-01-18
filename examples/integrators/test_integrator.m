addpath env
%% Create the system
IS = integrator(6);
IS = init_control(IS);

%IS = IS.run_deterministic(IS.controller);
tic
IS = IS.run_open_loop(IS.controller);
toc