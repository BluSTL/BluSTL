function Sys = STLC_apply_input(Sys)

it = Sys.system_data.time_index;
sys_time = [Sys.time(it) Sys.time(it+1)];

w_new = get_disturbance(Sys);

u_new = interp1(Sys.model_data.time(1:end-1),Sys.model_data.U', sys_time', 'previous' )';
x0 = Sys.system_data.X(:,it);

[Y,T,X] = system_step(Sys, u_new, sys_time, x0, w_new);

Sys.system_data.X(:,it:it+1)= X';
Sys.system_data.Y(:,it:it+1)= Y';
Sys.system_data.U(:,it:it+1)= u_new;
Sys.system_data.W(:,it:it+1)= w_new;
Sys.system_data.time(it:it+1) = sys_time; 
Sys.system_data.time_index=it+1;

end
