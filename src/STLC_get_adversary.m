function adversary = STLC_get_adversary(Sys)
% STLC_get_adversary    constructs the controller object for the adversarial 
%                       environment of an STLC_lti instance
%                           
% Input: 
%       Sys: an STLC_lti instance
%
% Output: 
%       adversary: a YALMIP optimizer object that solves the STL-constrained 
%                  optimal control problem for the adversarial environment
%                  in which Sys operates
%
% :copyright: TBD
% :license: TBD

%% Time
ts=Sys.ts; % sampling time
L=Sys.L;  % horizon (# of steps)

%% System dimensions and variables
nu=Sys.nu;
nx=Sys.nx;
nw=Sys.nw;
ny=Sys.ny;

% variables
X = sdpvar(nx, 2*L);
Y = sdpvar(ny, 2*L-1);
W = sdpvar(nw, 2*L);

% given as parameters to the optimizer
done = binvar(1,2*L-1);
p = sdpvar(1,L);
Xdone = sdpvar(nx, 2*L);
U = sdpvar(nu,2*L-1); 

Wref = sdpvar(nw, 2*L);

%% STL formula 
Fstl=[]; 
varStd = struct('X',X,'Y',Y,'U',U, 'W', W);

if isstruct(Sys.var)
    %remove overlapping fields from std
    var = rmfield(varStd, intersect(fieldnames(Sys.var), fieldnames(varStd)));
    keys = [fieldnames(var); fieldnames(Sys.var)];
    var = cell2struct([struct2cell(varStd); struct2cell(Sys.var)], keys, 1);
else
    var = varStd;
end
var.Wref = Wref;

stl_list= STLC_parse_stl_labels(Sys);
M = Sys.bigM;

Pphi=sdpvar(1,1);
for i = 1:numel(stl_list)
    phi = STLformula('phi', stl_list{i});
    [Fphi, Pphi] = STL2MILP_robust(phi, [1:L], 2*L, ts, var, M);
    Fstl = [Fstl Fphi];
end

%% Disturbances constraints
Fw = [];
for iw = 1:nw
        Fw = [Fw, Wref(iw,:) + Sys.w_lb(iw)  <= W(iw,:) <= Wref(iw,:)+Sys.w_ub(iw)];
end


%% Dynamics constraints
Fdyn = [];

[Ad,Bd,Cd,Dd]=ssdata(Sys.sysd);

Bdu=Bd(:,1:nu);
Bdw=Bd(:,nu+1:end);
Ddu=Dd(:,1:nu);
Ddw=Dd(:,nu+1:end);

% Constraints for states 
for k=1:2*L
    if k==1
        Fdyn = [Fdyn, X(:,1)==Xdone(:,1)];
    else
        % done values (history)
        % if k is past (done(k)==1), use values in Tdone, otherwise use linear update
        Fdyn = [Fdyn, Xdone(:,k) - (1-done(k-1))*M <=  X(:,k) <= Xdone(:, k)+ (1-done(k-1))*M];
        
        % not done values
        Fdyn = [Fdyn, ((Ad*X(:,k-1) + Bdu*U(:,k-1) + Bdw*W( :, k-1 )) - done(k-1)*M) <=  X(:,k) <= ((Ad*X(:,k-1) + Bdu*U(:,k-1) + Bdw*W( :, k-1 )) + done(k-1)*M)];
    end
end

% Constraints for outputs
for k=1:2*L-1
        Fdyn = [Fdyn, Y(:,k) == Cd*X(:,k)+ Ddu*U(:,k) + Ddw * W(:,k)];
end

%% Objective function
obj = min(Pphi(1,1:end-1));

options = Sys.solver_options;
adv_params= {done, p, Xdone, U, Wref};
adv_output= {W, X, Pphi};
adversary = optimizer([Fdyn Fw Fstl],obj,options,adv_params, adv_output );
