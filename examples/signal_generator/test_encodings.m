clear;
close all;

%% Create the system
nu = 2;
nw = 2;
SG = signal_generator(nu,nw);

%% Controller Initialisation
% Time
SG.time = 0:1:100; % time for the dynamics
SG.ts=1; % sampling time for controller
SG.L=4;  % horizon (# of steps)
SG.nb_stages=1; % repeats time

% Bounds
SG.u_ub(:) = 10;
SG.u_lb(:) = -10;
SG.lambda_rho = 100;

SG.Wref = [ cos(SG.time) ; 0.0*cos(SG.time)+1.1;];

phis= { '(w1(t)>3) or (w2(t)>3)', ...    
    'ev_[0,1.1] (w1(t)>0)', ...
    '((w1(t)>0)) or (ev_[0,1.1] (w1(t)>0))',...    
    'alw_[0,5] (w1(t)>0 )',...
    'not (alw_[0,5] (w1(t)>0))',...
    'alw_[0,1.1] (w1(t)>0)',...
    'alw( (y1(t)<1  =>  ev_[0, 2] (y1(t)>3)) and ((y1(t)>3)  =>  ev_[0, 2] (y1(t)<2)) )',...
    'ev_[0,10] (y1(t) > 3)';    
    }; 


for i = 5
    phi = phis{i};

    tic;
    rob = monitor(SG, phi, 'boolean');
    toc;
    
    tic;
    rob = monitor(SG, phi, 'robust');
    toc;
    
    tic;
    rob = monitor(SG, phi, 'interval');
%    toc;
    
end
