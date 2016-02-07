% Quadrotor linearized about hover (van der Burg 13)
% x = (p,v, r,w),     u = (uf,ux,uy)
% p = position (m),   v = velocity (m/s)
% r = attitude (rad), w = angular velocity (rad/s)
%
% Model adapted from LTLOpt (http://www.cds.caltech.edu/~ewolff/ltlopt.html)
%
%
% Input: 'dt' timestep (sec)
%        'N' length of traj
%        'dim' workspace dimension (2D or 3D)
% Output: 'X' sdpvar for state
%         'U' sdpvar for control
%         'F' dynamic constraints
%         'sys' system info


% Parameters (modified from van der Burg 13)
g = 9.81;       % gravity (m/s^2)
m = 1;          % mass (kg)
arm = 0.25;       % length of rotor arm (m)
J = 20*m*arm^2;   % moment of inertia (kg m^2) (est)

tmp = [0 g; -g 0; 0 0];
A = [zeros(3),   eye(3),     zeros(3,2), zeros(3,2); 
      zeros(3),   zeros(3),   tmp,        zeros(3,2); 
      zeros(2,3), zeros(2,3), zeros(2),   eye(2);
      zeros(2,3), zeros(2,3), zeros(2),   zeros(2)];


tmp = [0; 0; 1/m];
B = [zeros(3,1) zeros(3,2) zeros(3,1); 
      tmp        zeros(3,2) zeros(3,1); 
      zeros(2,1) zeros(2,2) zeros(2,1); 
      zeros(2,1) arm/J*eye(2) zeros(2,1)];
  
sys = ss(A,B,[],[]);


%% INITIALIZATION

%% Time
inits.ts=0.5; % sampling time (sec)
inits.L=10;  % horizon (# of steps)
inits.time = 0:1:200; % time for the date
inits.nb_stages=1;

%% System dimensions and variables
inits.nx = 10;    % num states
inits.nu = 3;     % num inputs
inits.nw = 1;     % dim disturbances
inits.dim = 3;  % dim env
inits.var = struct();

%% Bounds
inits.u_lb=-Inf;
inits.u_ub=Inf;
inits.u_delta=Inf;   

%% Create the STL specifications
inits = getSpecs(inits);

%% Disturbance signal
w = 0*ones(1,numel(inits.time));
Wref = w;

%% AUXILIARY SIGNALS
Aux = [];

inits.legends = [];
  
%% Initial state
X0=[0.5; 0.5; 0.0; zeros(7,1)];

%% SAVE
addpath('../../src')
warning('off','YALMIP:strict');

save('Quad_data.mat');
