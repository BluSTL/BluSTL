function [Sys, status] = STLC_compute_input(Sys, controller)
% STLC_compute_input
%
% Input:
%       Sys: an STLC_lti instance
%       controller: a YALMIP optimizer object representing the system's
%                   optimization problem
%
% Output:
%       Sys: modified with additional model_data
%       params: controller data
%
% :copyright: TBD
% :license: TBD

%% Stuff
nu=Sys.nu;
nx=Sys.nx;
min_rob = Sys.min_rob;
x0 = Sys.system_data.X(:,end);
M = Sys.bigM; % big M

%% Time
ts=Sys.ts; % sampling time
L=Sys.L;  % horizon (# of steps)
t_model = floor(Sys.time(Sys.system_data.time_index)/Sys.ts)+1;
Sys.model_data.time = ((0:2*L-1)+max(t_model-L,0))*ts;
i_transient = min(t_model,L);

%% test whether the input has already been computed
if (Sys.model_data.time_index<t_model)||(~isempty(Sys.model_data.Wbad))
    Sys.model_data.time_index = t_model;
    
    %% Sensing -- maybe we should call this predict_disturbance
    if isempty(Sys.model_data.Wbad)
        Wn = sensing(Sys);
    else % a disturbance has been obtained
        Wn= Sys.model_data.Wbad;
    end
    
    %% Initialize discrete data for the controller and environment
    donen = zeros(1,2*L-1); % done(1:k) = 1 iff everything has been computed up to step k
    pn = -1*M*ones(1,L);    % for activating robustness constraints
    Un = zeros(nu,2*L-1);
    Xn = zeros(max(nx,1),2*L);
    
    if i_transient==1
        Xn(:,1) = x0;
        pn(1) = min_rob;
    else%if i_transient<L
        donen(1:i_transient-1) = 1;  % we reuse what has been computed at the previous step
        Un = interp1(Sys.system_data.time, Sys.system_data.U',Sys.model_data.time(1:end-1)',[],0)';
        Xn = interp1(Sys.system_data.time, Sys.system_data.X',Sys.model_data.time',[],0)';
        pn(1:i_transient) = min_rob;
    end
    
    %% call solver
    [sol_control, errorflag1] = controller{{donen,pn,Xn,Un,Wn}};
    if(errorflag1==0)
        Sys.model_data.U = double(sol_control{1});
        Sys.model_data.X = double(sol_control{2});
        Sys.model_data.rob = double(sol_control{3});
        Sys.model_data.Y = [Sys.sysd.C*Sys.model_data.X(:,1:end-1) + Sys.sysd.D*[Sys.model_data.U; double(Wn(:,1:end-1))]];
        Sys.model_data.W = Wn;
        Sys.model_data.done = donen;
        Sys.model_data.p = pn;
    elseif (errorflag1==1 || errorflag1==15||errorflag1==12)  % some error, infeasibility or else
        disp(['Yalmip error (disturbance too bad ?): ' yalmiperror(errorflag1)]); % probably there is no controller for this w
    else
        disp(['Yalmip error: ' yalmiperror(errorflag1)]); % some other error
    end
    status = errorflag1;

%% else do nothing    
else
    status=0;
end


end


