function [Sys, params] = STLC_run_open_loop_adv(Sys, controller, adversary)
% STLC_run_open_loop_adv    runs an open-loop optimal control problem 
%                           for the system described by Sys, using the
%                           provided controller and adversary optimizer
%                           objects over the horizon defined by Sys.L
%                           
% Input: 
%       Sys: an STLC_lti instance
%       controller: a YALMIP opmitizer object representing the system's 
%                   optimization problem
%       adversary: a YALMIP opmitizer object representing the adversarial  
%                  environment's optimization problem
%
% Output: 
%       Sys: modified with additional system_data
%       params: controller data
%
% :copyright: TBD
% :license: TBD


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

% Disturbance
if isempty(Sys.Wref)
    Sys.Wref = 0*time;
end
Wref = Sys.Wref;
for iwx=1:nw
    Wrefn(iwx,:) = interp1( time , Wref(iwx,:)', time_d)';
end
Wn = Wrefn;

% Initialize discrete data for the controller and environment
M = Sys.bigM; % big M

donen = zeros(1,2*L-1); % done(1:k) = 1 iff everything has been computed up to step k
pn = -1*M*ones(1,L);    % for activating robustness constraints
Un = zeros(nu,2*L-1);
Xn = zeros(max(nx,1),2*L);
if (nx>0)
    Xn(:,1) = x0;      % only X0 is already computed
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

% call solver

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
u_new = Upred(:,1);
w_new = Wn(:,1);
[x_new, y_new] =  system_step(Sys, x0, u_new, w_new);
update_hist_data();
Sys = update_plot(Sys);

k=1;
while (k < 2*L-1)
    % pause;
    k = k+1;
    x0 = x_new;
    time_new = time_new+ts;
    u_new = Upred(:,k);
    w_new = Wn(:,k);
    [x_new, y_new] =  system_step(Sys, x0, u_new, w_new);
       
    %% Update plots
    update_hist_data();
    Sys= update_plot(Sys);
 
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
            else
                disp(['Yalmip error: ' yalmiperror(errorflag1)]); % some other error
            end
            
            % adversary's turn
            [sol_adversary, errorflag2] = adversary{{donen, pn, Xpred, Upred, Wrefn}}; %FIXME: what needs to go in here depends on the output of yalmip
            
            Wn = sol_adversary{1};
            
            wcrob = sol_adversary{3};
            fprintf('worst case robustness found: %g\n', wcrob(1));
            
            if(errorflag2==0 && wcrob(1) <= 0)  % found a bad disturbance
                disp(['Yalmip: ' 'Found a bad disturbance'])
            elseif(errorflag2==0 && wcrob(1) > 0) % no disturbance can violate specs
                fprintf('Control is good\n');
                break;
            elseif (errorflag2==1 || errorflag2==15||errorflag2==12)  % some error, infeasibility or else
                disp(['Yalmip error 1: ' yalmiperror(errorflag2)]);
                break
            else
                disp(['Yalmip error 2: ' yalmiperror(errorflag2)]); % some other error
                break
            end
        end
        params{end+1} = {time_d,donen,pn,Xn,Un,Wn};
                   
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

end

function Stop()
global StopRequest;
StopRequest = true;
end

