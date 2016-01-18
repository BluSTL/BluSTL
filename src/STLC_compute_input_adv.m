function [Sys, status_u, status_w] = STLC_compute_input_adv(Sys, controller, adversary)

iter=0;
status_w = [];
[Sys, status_u] = compute_input(Sys,controller);
cont = (iter<Sys.max_react_iter)&&(status_u==0);
while (cont)
    [Sys, status_w] = compute_disturbance(Sys,adversary);
    if status_w==0 % we found a bad disturbance 
        cont=1;
    elseif status_w==1; % control is good 
        cont=0;
    elseif status_w==-1; % some issue with compute_disturbance
        cont=0;
    end
        
    iter= iter+1;
    cont = cont&&(iter<Sys.max_react_iter);
    [Sys, status_u] = compute_input(Sys,controller);    
end