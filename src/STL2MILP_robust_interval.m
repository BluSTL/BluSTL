function [F,Plow,Pup] = STL2MILP_robust_interval(phi,kList,kMax,ts,var,M)
% STL2MILP_robust_interval  constructs MILP constraints in YALMIP that compute
%                           the robust interval of satisfaction, i.e. the 
%                           lower and upper bounds on the robustness of 
%                           satisfaction for specification phi
%
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
%           upper (Plow) and lower (Pup) bounds on quantitative satisfaction 
%           of phi over each time step in kList
%
% :copyright: TBD
% :license: TBD


    if (nargin==4);
        M = 1000;
    end;
        
    F = [];
    Plow = [];
    Pup = [];
    
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
            [F,Plow,Pup] = pred(phi.st,kList,var,M);
                     
        case 'not'
            [Frest,Prest_low,Prest_up] = STL2MILP_robust_interval(phi.phi,kList,kMax,ts, var,M);
            [Fnot, Pnot_low,Pnot_up] = not(Prest_low,Prest_up);
            F = [F, Fnot, Frest];
            Plow = Pnot_low;
            Pup = Pnot_up;

        case 'or'
            [Fdis1,Pdis1_low,Pdis1_up] = STL2MILP_robust_interval(phi.phi1,kList,kMax,ts, var,M);
            [Fdis2,Pdis2_low,Pdis2_up] = STL2MILP_robust_interval(phi.phi2,kList,kMax,ts, var,M);
            [For, Por_low,Por_up] = or([Pdis1_low;Pdis2_low],[Pdis1_up;Pdis2_up],M);
            F = [F, For, Fdis1, Fdis2];
            Plow = Por_low;
            Pup = Por_up;

        case 'and'
            [Fcon1,Pcon11,Pcon12] = STL2MILP_robust_interval(phi.phi1,kList,kMax,ts, var,M);
            [Fcon2,Pcon21,Pcon22] = STL2MILP_robust_interval(phi.phi2,kList,kMax,ts, var,M);
            [Fand,Pand1,Pand2] = and([Pcon11;Pcon21],[Pcon12;Pcon22],M);
            F = [F, Fand, Fcon1, Fcon2];
            Plow = Pand1;
            Pup = Pand2;

        case '=>'
            [Fant,Pant1,Pant2] = STL2MILP_robust_interval(phi.phi1,kList,kMax,ts,var,M);
            [Fcons,Pcons1,Pcons2] = STL2MILP_robust_interval(phi.phi2,kList,kMax,ts,var,M);
            [Fnotant,Pnotant1,Pnotant2] = not(Pant1,Pant2);
            [Fimp,PimPlow,PimPup] = or([Pnotant1;Pcons1],[Pnotant2;Pcons2],M);
            F = [F, Fant, Fnotant, Fcons, Fimp];
            Plow = PimPlow;
            Pup = PimPup;
            
        case 'always'
            kListAlw = unique(cell2mat(arrayfun(@(k) {k + a: k + b}, kList)));
            [Frest,Prest1,Prest2] = STL2MILP_robust_interval(phi.phi,kListAlw,kMax,ts,var,M);
            [Falw,Palw1,Palw2] = always(Prest1,Prest2,a,b,kList,kMax,M);
            F = [F, Falw];
            F = [F, Frest];
            Plow = Palw1;
            Pup = Palw2;

        case 'eventually'
            kListEv = unique(cell2mat(arrayfun(@(k) {k + a: k + b}, kList)));
            [Frest,Prest1,Prest2] = STL2MILP_robust_interval(phi.phi,kListEv,kMax,ts,var,M);
            [Fev,Pev1,Pev2] = eventually(Prest1,Prest2,a,b,kList,kMax,M);
            F = [F, Fev];
            F = [F, Frest];
            Plow = Pev1;
            Pup = Pev2;
          
        case 'until'
            [Fp,PPlow,PPup] = STL2MILP_robust_interval(phi.phi1,kList,kMax,ts,var,M);
            [Fq,Pq1,Pq2] = STL2MILP_robust_interval(phi.phi2,kList,kMax,ts,var,M);
            [Funtil,Puntil1,Puntil2] = until(PPlow,PPup,Pq1,Pq2,a,b,kList,kMax,M);
            F = [F,Funtil,Fp,Fq];
            Plow = Puntil1;
            Pup = Puntil2;
    end   
end

function [F,z1,z2] = pred(st,kList,var,M)
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
        t_st = regexprep(t_st,'\<t\>', num2str(kList(l)));
        try 
            z_eval = eval(t_st);
        end
        zl = sdpvar(1,1);
        F = [F, zl <= z_eval];
        z = [z,zl];
    end
    
    z1 = z;
    z2 = z;
  
end

% BOOLEAN OPERATIONS

function [F,Plow,Pup] = and(p_list1,p_list2,M)
    [F,Plow,Pup] = min_r(p_list1,p_list2,M);
end


function [F,Plow,Pup] = or(p_list_low,p_list_up,M)
     [F,Plow,Pup] = max_r(p_list_low,p_list_up,M);
end


function [F,Plow,Pup] = not(p_list1,p_list2)
    k = size(p_list1,2);
    m = size(p_list1,1);
    Plow = sdpvar(m,k);
    Pup = sdpvar(m,k);
    F = [Plow == -p_list2, Pup == -p_list1];
end



% TEMPORAL OPERATIONS

function [F,P_alwlow,P_alwup] = always(Plow,Pup,a,b,kList,kMax,M)
    F = [];
    k = size(kList,2);
    P_alwlow = sdpvar(1,k);
    P_alwup = sdpvar(1,k);
    kListAlw = unique(cell2mat(arrayfun(@(k) {k + a : k + b}, kList)));
    
    for i = 1:k
        [ia, ib, over] = getIndices(kList(i),a,b,kMax);
        ia_real = find(kListAlw==ia);
        ib_real = find(kListAlw==ib);
        [F0,P0low,P0up] = and(Plow(ia_real:ib_real)',Pup(ia_real:ib_real)',M);
        if over >=1
            F0 = [F0, P_alwlow(i) == -M];
        else
            F0 = [F0, P_alwlow(i)==P0low];
        end
        if over == 2
            F0 = [F0, P_alwup(i) == M];
        else
            F = [F;F0,P_alwup(i)==P0up];
        end
        
    end
    
end


function [F,P_evlow,P_evup] = eventually(Plow,Pup,a,b,kList,kMax,M)
    F = [];
    k = size(kList,2);
    P_evlow = sdpvar(1,k);
    P_evup = sdpvar(1,k);
    kListEv = unique(cell2mat(arrayfun(@(k) {k + a : k + b}, kList)));
    
    for i = 1:k
        [ia, ib, over] = getIndices(kList(i),a,b,kMax);
        ia_real = find(kListEv==ia);
        ib_real = find(kListEv==ib);
        [F0,P0low,P0up] = or(Plow(ia_real:ib_real)',Pup(ia_real:ib_real)',M);
        if over >=1
            F0 = [F0, P_evup(i) == M];
        else
            F0 = [F0, P_evup(i)==P0up];
        end
        if over == 2
            F0 = [F0, P_evlow(i) == -M];
        else
            F = [F;F0,P_evlow(i)==P0low];
        end
    end
    
end


function [F,P_until1,P_until2] = until(PPlow,PPup,Pq1,Pq2,a,b,k,M)
    
    F = [];
    P_until1 = sdpvar(1,k);
    P_until2 = sdpvar(1,k);
    
    for i = 1:k
        [ia, ib] = getIndices(i,a,b,k);
        F0 = []; 
        P0low = [];
        P0up = [];
        for j = ia:ib
            [F1,Plow1,Plow2] = until_mins(i,j,PPlow,PPup,Pq1,Pq2,M);
            F0 = [F0, F1];
            P0low = [P0low,Plow1];
            P0up = [P0up,Plow2];
        end
        [F4,P41,P42] = max_r(P0low,P0up);
        F = [F;F0,F4,P_until1(i)==P41,P_until2(i)==P42];
    end
    
end


% UTILITY FUNCTIONS

function [F,Plow,Pup] = min_r(p_list1,p_list2,M)
    
    k = size(p_list1,2);
    m = size(p_list1,1);
    
    Plow = sdpvar(1,k);
    z1 = binvar(m,k);
    
    Pup = sdpvar(1,k);
    z2 = binvar(m,k);
     
    F = [sum(z1,1) == ones(1,k),sum(z2,1) == ones(1,k)];
    for t=1:k
        for i=1:m
            F = [F, Plow(t) <= p_list1(i,t)];     
            F = [F, p_list1(i,t) - (1-z1(i,t))*M <= Plow(t) <= p_list1(i,t) + (1-z1(i,t))*M];
            
            F = [F, Pup(t) <= p_list2(i,t)];     
            F = [F, p_list2(i,t) - (1-z2(i,t))*M <= Pup(t) <= p_list2(i,t) + (1-z2(i,t))*M];
            
        end
    end
end

function [F,Plow,Pup] = max_r(p_list1,p_list2,M)

    k = size(p_list1,2);
    m = size(p_list1,1);
    
    Plow = sdpvar(1,k);
    z1 = binvar(m,k);
    
    Pup = sdpvar(1,k);
    z2 = binvar(m,k);
    
    F = [sum(z1,1) == ones(1,k),sum(z2,1) == ones(1,k)];
    for t=1:k
        for i=1:m
            F = [F, Plow(t) >= p_list1(i,t)];     
            F = [F, p_list1(i,t) - (1-z1(i,t))*M <= Plow(t) <= p_list1(i,t) + (1-z1(i,t))*M];
            
            F = [F, Pup(t) >= p_list2(i,t)];     
            F = [F, p_list2(i,t) - (1-z2(i,t))*M <= Pup(t) <= p_list2(i,t) + (1-z2(i,t))*M];        
        end
    end
end

function [F,Plow,Pup] = until_mins(i,j,PPlow,PPup,Pq1,Pq2,M)
    [F0,P0low,P0up] = min_r(PPlow(i:j)',PPup(i:j)',M);
    [F1,Plow,Pup] = min_r([Pq1(j),P0low],[Pq2(j),P0up],M);
    F = [F0,F1];
end

function [ia, ib, over] = getIndices(i,a,b,k)
    ia = min(k,i+a);
    ib = min(k,i+b);
    if k < i + b
        if k < i + a
            over = 2;
        else
            over = 1;
        end
    else
        over = 0;
    end
end


    
