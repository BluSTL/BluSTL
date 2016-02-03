clear;

A = [0 1 ;
     0 0];
Bu = [0;1];

Bw = [0 0;0 0]; 

C = [1 0];
Du = 0;
Dw = [0 1];

Sys= STLC_lti(A,Bu,Bw,C,Du,Dw); 

Sys.x0= [0 ; 0];

Sys.time = 0:.05:20; 
Sys.ts=.2; % sampling time for controller
Sys.L=10;   % horizon is 2s in that case
Sys.Wref = [sin(Sys.time);
            0*Sys.time];

Sys.u_ub = 10;  % upper bound on u 
Sys.u_lb = -10; % lower bound on u


Sys.stl_list = {'alw (ev_[0,1] (abs(y1(t)-w1(t)) < 0.1))'};

controller = get_controller(Sys)
Sys.plot_x =[]; % default was Sys.plot_x = [1 2]

Sys.solver_options = sdpsettings(Sys.solver_options, 'verbose', 2)
Sys= Sys.run_deterministic(controller);