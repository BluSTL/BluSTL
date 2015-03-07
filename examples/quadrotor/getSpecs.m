function [inits] = getSpecs( inits, dim, ex)

%% Create the environment

env = envLTL(ex);

% POLYTOPE CONSTRAINTS
% Control constraints
inits.var.Hu = [eye(3); -eye(3)];
inits.var.Ku = 0.04*[9.935; 3.62; 3.62;  
           4.545; 3.62; 3.62];
      
% State constraints
p = [0, 10]; % m
v = Inf*[-1, 1]; % m/s
r = Inf*[-1, 1]; % rad
w = Inf*[-1; 1]; % rad/s
inits.var.Hx = [eye(10); -eye(10)];
inits.var.Kx = [ p(2)*ones(3, 1);  v(2)*ones(3, 1);  r(2)*ones(2, 1);  w(2)*ones(2, 1);
      -p(1)*ones(3, 1); -v(1)*ones(3, 1); -r(1)*ones(2, 1); -w(1)*ones(2, 1)];

  
%fPoly = {'alw_[0,Inf] ((var.Hx*X(:,t) <= var.Kx))','alw_[0,Inf] ((var.Hu*U(:,t-1) <= var.Ku))'};
%fPoly = {'alw_[0,Inf] ((var.Hx*X(:,t) <= var.Kx))'};
fPoly = {};

% SAFETY
if 3 == dim
    inits.var.C = [1 zeros(1, 9); 
             0 1 zeros(1, 8);
             0 0 1 zeros(1, 7)];
elseif 2 == dim
    inits.var.C = [1 zeros(1, 9); 
             0 1 zeros(1, 8)];
end

G = env.work.unsafe;
fSafe = {};
if ~isempty(G)
    % Create safety constraints -- disjunction of polytopes
    nPoly = length(G);
    inits.var.HG = cell(1, nPoly);
    inits.var.KG = cell(1, nPoly);
    for i = 1:nPoly
        [h, l] = double(G(i));
        inits.var.HG{i} = h;
        inits.var.KG{i} = l;
        fSafe = [fSafe, sprintf('alw_[0,Inf](var.HG{%d}*var.C*X(:,t) > var.KG{%d})',i,i)];
    end
else
    fSafe = {};
end
fSafe = {};

if ex==3
    F =  {env.AP.a, env.AP.b, env.AP.c};
else
    F =  {env.AP.a, env.AP.b, env.AP.c, env.AP.d, env.AP.e};
end


if ~isempty(F)
    % Create "goal" constraints
    fGoal = {};
    inits.var.HF = cell(1,length(F));
    inits.var.KF = cell(1,length(F));
    
    for i = 1:length(F)
        nPoly = length(F{i});   

        inits.var.HF{i} = cell(1, nPoly);
        inits.var.KF{i} = cell(1, nPoly);
        
        fGoal{i} = '(ev_[0,Inf] (';
        for j = 1:nPoly
            [h,k] = double(F{i}(j));
            inits.var.HF{i}{j} = h;
            inits.var.KF{i}{j} = k;
            if j~=1
                fGoal{i} = [fGoal{i}, ' or ('];
            end
            fGoal{i} = [fGoal{i}, sprintf('(var.HF{%d}{%d}*var.C*X(:,t) <= var.KF{%d}{%d})',i,j,i,j)];
           
        end
        for j = 1:nPoly-1
            fGoal{i} = [fGoal{i}, ')'];
        end
        fGoal{i} = [fGoal{i}, '))'];
    end
else
    fGoal = {};
end
%fGoal={};

%% STL formula
inits.stl_list = [fPoly, fSafe, fGoal{:}];

end

