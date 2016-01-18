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

Sys.time = 0:.01:10; 
Sys.ts=.1; % sampling time for controller
Sys.L=10;   % horizon is 2s in that case
Sys.Wref = [.05*cos(10*Sys.time);
            0*Sys.time];

Sys.u_ub = 20;  % upper bound on u 
Sys.u_lb = -20; % lower bound on u


Sys.stl_list = {'ev_[0,2] alw_[0,2] ( abs(y1(t)-w1(t)) < 0.1)'};

controller = get_controller(Sys, 'interval')

Sys= Sys.run_deterministic(controller);