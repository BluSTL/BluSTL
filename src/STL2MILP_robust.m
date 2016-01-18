function [F,P] = STL2MILP_robust(phi,kList,kMax,ts,var,M)
% STL2MILP_robust  constructs MILP constraints in YALMIP that compute
%                  the robustness of satisfaction for specification phi
%
% Input: 
%       phi:    an STLformula
%       kList:  a list of time steps at which the formula is to be enforced
%       kMAx:   the length of the trajectory
%       ts:     the interval (in seconds) used for discretizing time
%       var:    a dictionary mapping strings to variables
%       M:   	a large positive constant used for big-M constraints  
%
% Output: 
%       F:  YALMIP constraints
%       P:  a struct containing YALMIP decision variables representing 
%           the quantitative satisfaction of phi over each time step in
%           kList
%
% :copyright: TBD
% :license: TBD

    if (nargin==4);
        M = 1000;
    end;
        
    F = [];
    P = [];
    
    if ischar(phi.interval)
        interval = [str2num(phi.interval)];
    else
        interval = phi.interval;
    end
    
    a = interval(1);
    b = interval(2);
        
    a = max([0 floor(a/ts)-1]); 
    b = ceil(b/ts)-1; 
    
    if b==Inf
        b = kMax;
    end
    
    switch (phi.type)
        
        case 'predicate'
            [F,P] = pred(phi.st,kList,var,M);
                     
        case 'not'
            [Frest,Prest] = STL2MILP_robust(phi.phi,kList,kMax,ts, var,M);
            [Fnot, Pnot] = not(Prest);
            F = [F, Fnot, Frest];
            P = Pnot;

        case 'or'
            [Fdis1,Pdis1] = STL2MILP_robust(phi.phi1,kList,kMax,ts, var,M);
            [Fdis2,Pdis2] = STL2MILP_robust(phi.phi2,kList,kMax,ts, var,M);
            [For, Por] = or([Pdis1;Pdis2],M);
            F = [F, For, Fdis1, Fdis2];
            P = Por;

        case 'and'
            [Fcon1,Pcon1] = STL2MILP_robust(phi.phi1,kList,kMax,ts, var,M);
            [Fcon2,Pcon2] = STL2MILP_robust(phi.phi2,kList,kMax,ts, var,M);
            [Fand, Pand] = and([Pcon1;Pcon2],M);
            F = [F, Fand, Fcon1, Fcon2];
            P = Pand;

        case '=>'
            [Fant,Pant] = STL2MILP_robust(phi.phi1,kList,kMax,ts,var,M);
            [Fcons,Pcons] = STL2MILP_robust(phi.phi2,kList,kMax,ts, var,M);
            [Fnotant,Pnotant] = not(Pant);
            [Fimp, Pimp] = or([Pnotant;Pcons],M);
            F = [F, Fant, Fnotant, Fcons, Fimp];
            P = [Pimp,P];
            
        case 'always'
            kListAlw = unique(cell2mat(arrayfun(@(k) {min(kMax,k + a): min(kMax,k + b)}, kList)));
            [Frest,Prest] = STL2MILP_robust(phi.phi,kListAlw,kMax,ts, var,M);
            [Falw, Palw] = always(Prest,a,b,kList,kMax,M);
            F = [F, Falw];
            P = [Palw, P];
            F = [F, Frest];

        case 'eventually'
            kListEv = unique(cell2mat(arrayfun(@(k) {min(kMax,k + a): min(kMax,k + b)}, kList)));
            [Frest,Prest] = STL2MILP_robust(phi.phi,kListEv,kMax,ts, var,M);
            [Fev, Pev] = eventually(Prest,a,b,kList,kMax,M);
            F = [F, Fev];
            P = [Pev, P];
            F = [F, Frest];
          
        case 'until'
            [Fp,Pp] = STL2MILP_robust(phi.phi1,kList,kMax,ts, var,M);
            [Fq,Pq] = STL2MILP_robust(phi.phi2,kList,kMax,ts, var,M);
            [Funtil, Puntil] = until(Pp,Pq,a,b,kList,kMax,M);
            F = [F, Funtil, Fp, Fq];
            P = Puntil;
    end
end

function [F,z] = pred(st,kList,var,M)
    % Enforce constraints based on predicates 
    % var is the variable dictionary    
        
    fnames = fieldnames(var);
    
    for ifield= 1:numel(fnames)
        eval([ fnames{ifield} '= var.' fnames{ifield} ';']); 
    end          
        
    st = regexprep(st,'\[t\]','\(t\)'); % Breach compatibility
    if strfind(st, '<')       
        tokens = regexp(st, '(.+)\s*<\s*(.+)','tokens');
        st = ['-(' tokens{1}{1} '- (' tokens{1}{2} '))']; 
    end
    if strfind(st, '>')
        tokens = regexp(st, '(.+)\s*>\s*(.+)','tokens');
        st= [ '(' tokens{1}{1} ')-(' tokens{1}{2} ')' ];
    end
         
    F = [];
    z = [];
    
    k = size(kList,2);
    
    for l=1:k
        
        t_st = st;
        t_st = regexprep(t_st,'\<t\>',num2str(kList(l)));
        try 
            z_eval = eval(t_st);
        end
        zl = sdpvar(1,1);
        F = [F, zl == z_eval];
        z = [z,zl];
    end
end


% BOOLEAN OPERATIONS

function [F,P] = and(p_list,M)
    [F,P] = min_r(p_list,M);
end


function [F,P] = or(p_list,M)
     [F,P] = max_r(p_list,M);
end


function [F,P] = not(p_list)
    k = size(p_list,2);
    m = size(p_list,1);
    P = sdpvar(1,k);
    F = [P(:) == -p_list(:)];
end


% TEMPORAL OPERATIONS

function [F,P_alw] = always(P, a,b,kList,kMax,M)
    F = [];
    k = size(kList,2);
    P_alw = sdpvar(1,k);
    kListAlw = unique(cell2mat(arrayfun(@(k) {min(kMax,k + a) : min(kMax,k + b)}, kList)));
    
    for i = 1:k
        [ia, ib] = getIndices(kList(i),a,b,kMax);
        ia_real = find(kListAlw==ia);
        ib_real = find(kListAlw==ib);
        [F0,P0] = and(P(ia_real:ib_real)',M);
        F = [F;F0,P_alw(i)==P0];
    end
    
end


function [F,P_ev] = eventually(P, a,b,kList,kMax,M)
    F = [];
    k = size(kList,2);
    P_ev = sdpvar(1,k);
    kListEv = unique(cell2mat(arrayfun(@(k) {min(kMax,k + a) : min(kMax,k + b)}, kList)));
    
    for i = 1:k
        [ia, ib] = getIndices(i,a,b,kMax);
        ia_real = find(kListEv==ia);
        ib_real = find(kListEv==ib);
        [F0,P0] = or(P(ia_real:ib_real)',M);
        F = [F;F0,P_ev(i)==P0];
    end
    
end


function [F,P_until] = until(Pp,Pq,a,b,k,M)
    
    F = [];
    P_until = sdpvar(1,k);
    
    for i = 1:k
        [ia, ib] = getIndices(i,a,b,kMax);
        F0 = []; 
        P0 = [];
        for j = ia:ib
            [F1,P1] = until_mins(i,j,Pp,Pq,M);
            F0 = [F0, F1];
            P0 = [P0, P1];
        end
        [F4,P4] = max_r(P0);
        F = [F;F0,F4,P_until(i)==P4];
    end
    
end


% UTILITY FUNCTIONS

function [F,P] = min_r(p_list,M)
    
    k = size(p_list,2);
    m = size(p_list,1);
    
    P = sdpvar(1,k);
    z = binvar(m,k);
     
    F = [sum(z,1) == ones(1,k)];
    for t=1:k
        for i=1:m
            F = [F, P(1,t) <= p_list(i,t)];     
            F = [F, p_list(i,t) - (1-z(i,t))*M <= P(t) <= p_list(i,t) + (1-z(i,t))*M];
        end
    end
end

function [F,P] = max_r(p_list,M)

    k = size(p_list,2);
    m = size(p_list,1);
    
    P = sdpvar(1,k);
    z = binvar(m,k);
    
    F = [sum(z,1) == ones(1,k)];
    for t=1:k
        for i=1:m
            F = [F, P(1,t) >= p_list(i,t)];     
            F = [F, p_list(i,t) - (1-z(i,t))*M <= P(t) <= p_list(i,t) + (1-z(i,t))*M];
        end
    end
end

function [F,P] = until_mins(i,j,Pp,Pq,M)
    [F0,P0] = min_r(Pp(i:j)',M);
    [F1,P] = min_r([Pq(j),P0],M);
    F = [F0,F1];
end

function [ia, ib] = getIndices(i,a,b,k)
    ia = min(k,i+a);
    ib = min(k,i+b);
end


    
