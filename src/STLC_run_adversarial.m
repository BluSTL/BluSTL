function [Sys, params] = STLC_run_adversarial(Sys, controller,adversary)
%
% run_adversarial(controller, adversary)
%
%  Output: hist_data, params (controller data)
%
%
global StopRequest
StopRequest=0;

rob = Sys.min_rob;

%% Time
sys = Sys.sys;
ts=Sys.ts; % sampling time
L=Sys.L;  % horizon (# of steps)
time = Sys.time; % time for the date
time_d = (0:2*L-1)*ts; % discretized time for the controller

%% System dimensions and variables
Sys.sysd = c2d(sys,ts);

x0 = Sys.x0;
nu=Sys.nu;
nx=Sys.nx;
nw=Sys.nw;
ny=Sys.ny;

% reference disturbance
if isempty(Sys.Wref)
    Sys.Wref = 0*time;
end
Wref = Sys.Wref;
for iwx=1:nw
    Wrefn(iwx,:) = interp1( time , Wref(iwx,:)', time_d)';
end
Wn = Wrefn;

%% Make data nb_stages times longer
nb_stages=Sys.nb_stages;
ntime = zeros(1, nb_stages*numel(time));
for istage = 0:nb_stages-1
    ntime(istage*numel(time)+1:(istage+1)*numel(time))= time+istage*(time(end)+time(2)) ;
end
time = ntime;
Wref = repmat(Wref,1,Sys.nb_stages);


%% Initialize discrete data for the controller and environment

M = Sys.bigM; % big M
donen = zeros(1,2*L-1); % done(1:k) = 1 iff everything has been computed up to step k
pn = -1*M*ones(1,L);    % for activating robustness constraints
Un = zeros(nu,2*L-1);
Xn = zeros(max(nx,1),2*L);
if (nx>0)
    Xn(:,1) = x0;           % only X0 is already computed
end
pn(1) = rob;

Upred = zeros(nu,2*L-1);
Xpred = zeros(nx,2*L);

u_new = [];
w_new = [];
x_new = [];
y_new = [];
time_new = 0;
params = {};

% call solver for the first horizon
i_transient=1;
i_past = 1;

%% Init system and model data
Sys.system_data=struct;
Sys.model_data=struct;

Sys.system_data.time = [];
Sys.system_data.U = [];
Sys.system_data.X = x0;
Sys.system_data.Y = [];
Sys.system_data.W = [];

Sys.model_data.time = time_d;
Sys.model_data.X = repmat(0*time_d(1:end), [nx 1]);
Sys.model_data.Y = repmat(0*time_d(1:end-1), [ny 1]);
Sys.model_data.U = repmat(0*time_d(1:end-1), [nu 1]);
Sys.model_data.W = Wn;

compute_input();

u_new = Upred(:,i_transient);
w_new =  compute_w();
[x_new, y_new] =  system_step(Sys, x0, u_new, w_new);
params{end+1} = {time_d,donen,pn,Xn,Un,Wn,Wrefn};
i_past =  i_past+1;

update_hist_data();
Sys = update_plot(Sys);

%% loop
pause
i_transient = i_transient+1;
while (time_d(end)+ts< time(end))
    % pause;
    x0 = x_new;
    time_new = time_new+ts;
    
    %% updates the model of controller and environment for the next horizon
    update_controller_data();
    
    %% compute input for the next horizon
    
    tic;
    compute_input();
    toc;
    %% update states
    u_new = Upred(:,i_transient);
    w_new = compute_w();
    
    [x_new, y_new] =  system_step(Sys, x0, u_new, w_new);
    params{end+1} = {time_d,donen,pn,Xn,Un,Wn,Wrefn};
    i_past =  i_past+1;

    %% Update plots
    update_hist_data();
    Sys= update_plot(Sys);
    
    if i_transient < L
        i_transient = i_transient+1;
    end
    drawnow;
    %  pause;
    if StopRequest
        break;
    end
end

    function compute_input()
        
        for i=1:Sys.max_react_iter
            
            % controller's turn
            [sol_control, errorflag1] = controller{{donen,pn,Xn,Un,Wn}};
            if(errorflag1==0)  % found a good control
                %disp(['Yalmip: ' yalmiperror(errorflag1)])
                Upred = sol_control{1};
                Xpred = sol_control{2};
            elseif (errorflag1==1 || errorflag1==15||errorflag1==12)  % some error, infeasibility or else
                disp(['Yalmip error (disturbance too bad ?): ' yalmiperror(errorflag1)]); % probably there is no controller for this w
                StopRequest=1;
                break
            else
                disp(['Yalmip error: ' yalmiperror(errorflag1)]); % some other error
                break
            end
            
            % adversary's turn
            [sol_adversary, errorflag2] = adversary{{donen, pn, Xpred, Upred, Wrefn}}; %FIXME: what needs to go in here depends on the output of yalmip
            
            Wn = sol_adversary{1};
            wcrob = sol_adversary{3};
            
           % fprintf('worst case robustness found: %g\n', wcrob(1));
            
            if(errorflag2==0 && wcrob(1) <= 0)  % found a bad disturbance
                disp(['Yalmip: ' 'Found a bad disturbance'])
            elseif(errorflag2==0 && wcrob(1) > 0) % no disturbance can violate specs
                %fprintf('Control is good\n');
                break;
            elseif (errorflag2==true || errorflag2==15||errorflag2==12)  % some error, infeasibility or else
                disp(['Yalmip error 1: ' yalmiperror(errorflag2)]);
                StopRequest=true;
                break
            else
                disp(['Yalmip error 2: ' yalmiperror(errorflag2)]); % some other error
                StopRequest=true;
                break
            end
        end
                
    end

    function update_hist_data()
        
        Sys.system_data.time(end+1) = time_new;
        Sys.system_data.U(:,end+1) = u_new;
        Sys.system_data.X(:,end+1) = x_new;
        Sys.system_data.Y(:,end+1) = y_new;
        Sys.system_data.W(:,end+1) = w_new;
        
        Sys.model_data.time = time_d;
        Sys.model_data.X = double(Xpred);
        Sys.model_data.Y = [Sys.sysd.C*double(Xpred(:,1:end-1)) + Sys.sysd.D*[double(Upred); double(Wn(:,1:end-1))]];
        Sys.model_data.U = double(Upred);
        Sys.model_data.W = Wn;
        
    end

    function update_controller_data()
        
        if (i_past>= L+1)
            time_d = time_d+ts; % move forward one time step
        end
        
        % Note: the following assumes that system and model data are
        % both sampled with ts - should be changed eventually
        
        % reinitialize disturbance to the reference
        for wx=1:nw
            Wrefn(wx,:) = interp1( time , Wref(wx,:)', time_d)';
        end
        Wn = Wrefn;
        
        if i_transient<L
            donen(1:i_transient-1) = 1;  % we reuse what has been computed at the previous step
            Un(:,1:i_transient-1) = Sys.system_data.U(:,1:i_transient-1); %  previously computed inputs
            Wn(:,1:i_transient-1) = Sys.system_data.W(:,1:i_transient-1);
            Xn(:,1:i_transient) = Sys.system_data.X(:, 1:i_transient);
            pn(i_transient) = rob;
        else
            pn(1:L) = rob*ones(1,L);
            donen(1:L-1) = 1;
            Un(:,1:L-1) = Sys.system_data.U(:,end-L+2:end);   %    previously computed inputs
            Xn(:,1:L) = Sys.system_data.X(:,end-L+1:end);     %    previously computed temperatures
            Wn(:,1:L-1) = Sys.system_data.W(:,end-L+2:end);
        end
        
        
        
    end

    function w_new= compute_w() 
        % computes a random disturbance between w bounds 
        dw = Sys.w_ub- Sys.w_lb;    
        w_new = Wrefn(:,i_transient)+ dw'.*(2*rand(Sys.nw,1)-1)/3;        
    end


end

function Stop()
global StopRequest;
StopRequest = true;
end

