clear;

A = [0 1 ;
     0 0];
Bu = [0;1];

Bw = [0 0;0 0]; 

C = [1 0];
Du = 0;
Dw = [-1 1];
Sys= STLC_lti(A,Bu,Bw,C,Du,Dw); 

Sys.x0=[0.1; 0.0566];
Sys.time = 0:.02:10; 

Sys.ts=.04; % sampling time for controller
Sys.L=10;   % horizon is 2s in that case
Sys.Wref = [.2*cos(Sys.time);
             0*Sys.time];

Sys.u_ub =  10;  % upper bound on u 
Sys.u_lb = -10; % lower bound on u

Sys.w_ub(:) = .01;   
Sys.w_lb(:) = -.01; 

Sys.plot_x = [1];
Sys.stl_list = {'alw ( (y1(t) < 0.2) and (-y1(t)<0.2))'};
Sys.min_rob = 0.;
Sys.lambda_rho = 100;
Sys.stop_button=1;
Sys.bigM = 1000;
Sys.solver_options.verbose = 1;

tic;
controller = get_controller(Sys);
toc;

tic;
adversary = get_adversary(Sys);
toc;

Sys = Sys.reset_data();
%Sys = run_open_loop_adv(Sys, controller, adversary); 
Sys = run_adversarial(Sys, controller, adversary); 

% Sys= compute_input(Sys,controller);
% Sys.update_plot();
% Sys= compute_disturbance(Sys,adversary);
% Sys.update_plot();
% Sys= compute_input(Sys,controller);
% Sys.update_plot();
% Sys= compute_disturbance(Sys,adversary);
% Sys.update_plot();


%Sys= Sys.run_open_loop(controller);
%Sys= Sys.run_deterministic(controller);
