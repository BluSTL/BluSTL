%%
% We can run our system in closed loop, but this is not very interesting,
% because w is 0 anyway. Let's change that 
Sys.Wref = Sys.time*0.;
Sys.Wref(30:40) = 1; 
Sys.Wref(60:80) = -0.5; 


%%
% and the specs:
Sys.stl_list = {'alw (ev_[0,2.] alw_[0,1] ( abs(y1(t)-w1(t)) < 0.1))'};
controller = get_controller(Sys);

%%
% This time we will only plot input and outputs, i.e., disable the state
% plotting:
Sys.plot_x =[]; % default was Sys.plot_x = [1 2]
run_deterministic(Sys, controller);

%%
% More examples are given in the folder BluSTL/examples. In particular the 
% hvac_room case study demonstrate the adversarial scenario, as well as
% plot customization. The idea is to create a class derived from STLC_lti
% and specialize the update_plot method. 

