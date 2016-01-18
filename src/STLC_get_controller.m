function controller = STLC_get_controller(Sys,enc)
% STLC_get_controller constructs the controller object for an STLC_lti instance
%                           
% Input: 
%       Sys: an STLC_lti instance
%
% Output: 
%       controller: a YALMIP optimizer object that solves the STL-constrained 
%                   optimal control problem for Sys
%
% :copyright: TBD
% :license: TBD

if nargin < 2
    enc='robust';
end

%% Time
ts=Sys.ts; % sampling time
L=Sys.L;   % horizon (# of steps)

%% System dimensions and variables
nu=Sys.nu;
nx=Sys.nx;
nw=Sys.nw;
ny=Sys.ny;

% variables
X = sdpvar(nx, 2*L); 
U = sdpvar(nu, 2*L-1);
Y = sdpvar(ny, 2*L);

% parameters 
W = sdpvar(nw, 2*L);
done = binvar(1,2*L-1);
p = sdpvar(1,L);
Udone = sdpvar(nu,2*L-1);
Xdone = sdpvar(nx, 2*L);

%% STL formula
Fstl = [];
Pstl = [];
Pstllow = [];
Pstlup = [];

varStd = struct('X',X,'Y', Y,'U',U, 'W', W);

if isstruct(Sys.var)
    %remove overlapping fields from std
    var = rmfield(varStd, intersect(fieldnames(Sys.var), fieldnames(varStd)));
    keys = [fieldnames(var); fieldnames(Sys.var)];
    var = cell2struct([struct2cell(varStd); struct2cell(Sys.var)], keys, 1);
else
    var = varStd;
end

stl_list= STLC_parse_stl_labels(Sys);
M = Sys.bigM;

for i = 1:numel(stl_list)
    phi = STLformula('phi', stl_list{i});
    switch enc
        case 'boolean'
            [Fphi, Pphi] = STL2MILP_boolean(phi, [1:L], 2*L, ts, var,M); 
            Pstl = [Pstl; Pphi];
        case 'robust'
            [Fphi, Pphi] = STL2MILP_robust(phi, [1:L], 2*L, ts, var,M);
            Pstl = [Pstl; Pphi];
        case 'interval'
            [Fphi, Pphilow, Pphiup] = STL2MILP_robust_interval(phi, [1:2*L], 2*L, ts, var,M); 
            Pstllow = [Pstllow; Pphilow];
            Pstlup = [Pstlup; Pphiup];
    end
    Fstl = [Fstl Fphi];
    
    % add constraints to enforce satisfaction
    switch enc
        case 'boolean'
            for j = 1:min(L, size(Pphi,2))
                if Sys.min_rob > 0
                    Fstl = [Fstl Pphi(:,j) == 1];
                end
            end
        case 'robust'
            for j = 1:min(L, size(Pphi,2))
                Fstl = [Fstl Pphi(:,j)>= p(j)]; % TODO this is specific to alw (phi), what about ev, until...
            end
        case 'interval'
            %Fstl = [Fstl Pphilow(:,1)>= p(1)]; % TODO this is specific to alw (phi), what about ev, until...
            for j = 1:min(L, size(Pphiup,2))
               Fstl = [Fstl Pphiup(:,j)>= p(j)]; % TODO this is specific to alw (phi), what about ev, until...
            end      
    end
end


%% Input constraints
Fu = [];

% Bounds
for iu = 1:nu
    Fu = [ Fu, Sys.u_lb(iu) <= U(iu,:) <= Sys.u_ub(iu)] ;  % bounds constraints on u
end

% Bounded variability
delta_not_inf = 0;
for iu = 1:nu
    dif = sdpvar(nu,2*L-2);
    F_dif = dif(:,1:2*L-2) == U(:,2:2*L-1) - U(:,1:2*L-2);
    
    if (Sys.u_delta(iu) < Sys.u_ub(iu)- Sys.u_lb(iu))
        Fdif = [F_dif, -Sys.u_delta(iu) <= dif <= Sys.u_delta(iu)];
        delta_not_inf = 1;
    end
    
end

if delta_not_inf
    Fu = [Fu Fdif];
end


%% Dynamics constraints
Fdyn = [];

[Ad,Bd,Cd,Dd]=ssdata(Sys.sysd);

Bdu=Bd(:,1:nu);
Bdw=Bd(:,nu+1:end);
Ddu=Dd(:,1:nu);
Ddw=Dd(:,nu+1:end);
K = Sys.K;

% Constraints for states (if any)
for k=1:2*L
    if k==1
        Fdyn = [Fdyn, X(:,1)==Xdone(:,1)];
    else
        % done values (history)
        % if k is past (done(k)==1), use values in Tdone, otherwise use linear update
        Fdyn = [Fdyn, Xdone(:,k) - (1-done(k-1))*M <=  X(:,k) <= Xdone(:, k)+ (1-done(k-1))*M];
        
        % not done values
        Fdyn = [Fdyn, ((Ad*X(:,k-1) + Bdu*U(:,k-1) + Bdw*W( :, k-1 ) + K) - done(k-1)*M) <=  X(:,k) <= ((Ad*X(:,k-1) + Bdu*U(:,k-1) + Bdw*W( :, k-1 )+ K) + done(k-1)*M)];
    end
end

% Constraints for inputs
for k=1:2*L
    if k>1
        Fdyn = [Fdyn, Udone(:,k-1) - (1-done(k-1))*M <=  U(:,k-1) <= Udone(:,k-1) + (1-done(k-1))*M];
    end
end

% Constraints for outputs (if any)
for k=1:2*L-1
        Fdyn = [Fdyn, Y(:,k) == Cd*X(:,k)+ Ddu*U(:,k) + Ddw *W(:,k)];
end

options = Sys.solver_options;
param_controller = {done, p, Xdone, Udone, W};
if strcmp(enc,'interval')
    output_controller =  {U,X,[Pstllow;Pstlup]};
else
    output_controller =  {U,X,Pstl};
end


if numel(stl_list) == 0
    Pstl = sdpvar(1,1);
end

%% Objective function
switch enc
    case 'boolean'
        obj = get_objective(Sys,X,Y,U);
    case 'robust'
        obj = get_objective(Sys,X,Y,U,W, Pstl(:,1:L-1), Sys.lambda_rho);
    case 'interval'
        obj = get_objective(Sys,X,Y,U,W, Pstllow, Sys.lambda_rho, Sys.lambda_t1);
end

controller = optimizer([Fdyn, Fstl, Fu],obj,options,param_controller, output_controller);


