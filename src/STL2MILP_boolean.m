 function [F,P] = STL2MILP_boolean(phi,kList,kMax,ts,var,M)
% STL2MILP_boolean  constructs MILP constraints in YALMIP that compute
%                   the boolean satisfaction for specification phi
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
%       P:  YALMIP decision variables representing the boolean satisfaction 
%           over each time step in kList
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
            [F,P] = pred(phi.st,kList,var);
            
        case 'not'
            [Frest,Prest] = STL2MILP_boolean(phi.phi,kList,kMax,ts, var,M);
            [Fnot, Pnot] = not(Prest);
            F = [F, Fnot, Frest];
            P = Pnot;

        case 'or'
            [Fdis1,Pdis1] = STL2MILP_boolean(phi.phi1,kList,kMax,ts, var,M);
            [Fdis2,Pdis2] = STL2MILP_boolean(phi.phi2,kList,kMax,ts, var,M);
            [For, Por] = or([Pdis1;Pdis2]);
            F = [F, For, Fdis1, Fdis2];
            P = Por;

        case 'and'
            [Fcon1,Pcon1] = STL2MILP_boolean(phi.phi1,kList,kMax,ts, var,M);
            [Fcon2,Pcon2] = STL2MILP_boolean(phi.phi2,kList,kMax,ts, var,M);
            [Fand, Pand] = and([Pcon1;Pcon2]);
            F = [F, Fand, Fcon1, Fcon2];
            P = Pand;

        case '=>'
            [Fant,Pant] = STL2MILP_boolean(phi.phi1,kList,kMax,ts, var,M);
            [Fcons,Pcons] = STL2MILP_boolean(phi.phi2,kList,kMax,ts, var,M);
            [Fnotant,Pnotant] = not(Pant);
            [Fimp, Pimp] = or([Pnotant;Pcons]);
            F = [F, Fant, Fnotant, Fcons, Fimp];
            P = Pimp;
            
        case 'always'
            kListAlw = unique(cell2mat(arrayfun(@(k) {min(kMax,k + a): min(kMax,k + b)}, kList)));
            [Frest,Prest] = STL2MILP_boolean(phi.phi,kListAlw,kMax,ts, var,M);
            [Falw, Palw] = always(Prest, a,b, kList,kMax);
            F = [F, Falw];
            P = [P, Palw];
            F = [F, Frest];

        case 'eventually'
            kListEv = unique(cell2mat(arrayfun(@(k) {min(kMax,k + a): min(kMax,k + b)}, kList)));
            [Frest,Prest] = STL2MILP_boolean(phi.phi,kListEv,kMax,ts, var,M);
            [Fev, Pev] = eventually(Prest, a,b, kList,kMax);
            F = [F, Fev];
            P = [P, Pev];
            F = [F, Frest];
          
        case 'until'
            [Fp,Pp] = STL2MILP_boolean(phi.phi1,kList,kMax,ts, var,M);
            [Fq,Pq] = STL2MILP_boolean(phi.phi2,kList,kMax,ts, var,M);
            [Funtil, Puntil] = until([Pp,Pq,a,b,k]);
            F = [F, Funtil, Fp, Fq];
            P = Puntil;
    end
end

function [F,z] = pred(st,kList,var)
    % Enforce constraints based on predicates 
    % var is the variable dictionary    
    
    fnames = fieldnames(var);
    
    for ifield= 1:numel(fnames)
        eval([ fnames{ifield} '= var.' fnames{ifield} ';']); 
    end          
        
    st = regexprep(st,'\[t\]','\(t\)'); % Breach compatibility ?
    if strfind( st, '<')
        tokens = regexp(st, '(.+)\s*<\s*(.+)','tokens');
        st = ['-(' tokens{1}{1} '- (' tokens{1}{2} '))']; 
    end
    if strfind(st, '>')
        tokens = regexp(st, '(.+)\s*>\s*(.+)','tokens');
        st= [ '(' tokens{1}{1} ')-(' tokens{1}{2} ')' ];
    end
    
    F = [];
    z = [];
    
    bigM = 1000;
    bigMst1 = 'bigM*z(:,t)';
    bigMst2 = 'bigM*(1 - z(:,t))';
  
    k = size(kList,2);
    
    for l=1:k
        
        t_st = st;
        t_st = regexprep(t_st,'\<t\>', num2str(kList(l)));

        % ADD VARIABLES
        zl = binvar(1,1);
        z = [z,zl];
        
        % ADD CONSTRAINTS
        
        bigM1 = regexprep(bigMst1,',t\)',[',',num2str(l) '\)']);
        t_st1 = [t_st '<=' bigM1];
        F = [F, eval(t_st1)];
        
        bigM2 = regexprep(bigMst2,',t\)',[',',num2str(l) '\)']);
        t_st2 = ['-(' t_st ') <= ' bigM2];
        F = [F, eval(t_st2)];
        
    end
end


function [F,P] = and(p_list)
    k = size(p_list,2);
    m = size(p_list,1);
    
    P = binvar(1,k);
    F = [];
    for t=1:k
        F = [F, P(t) <= p_list(:,t)];        
        F = [F, P(t) >= 1 - m + sum(p_list(:,t),1)];
    end
end

function [F,P] = or(p_list)
    k = size(p_list,2);
    m = size(p_list,1);
    
    P = binvar(1,k);
    F = [];
    for t=1:k
        F = [F, P(t) >= p_list(:,t)];        
        F = [F, P(t) <= sum(p_list(:,t),1)];
    end

end

function [F,P] = not(p_list)
    k = size(p_list,2);
    m = size(p_list,1);
    assert( m == 1 )
    P = binvar(1,k);
    F = [P(:) == 1 - p_list(:)];
end


function [F,P_alw] = always(P, a,b,kList,kMax)
    F = [];
    k = size(kList,2);
    P_alw = sdpvar(1,k);
    kListAlw = unique(cell2mat(arrayfun(@(k) {min(kMax,k + a) : min(kMax,k + b)}, kList)));    
    for i = 1:k
        [ia, ib] = getIndices(kList(i),a,b,kMax);
        ia_real = find(kListAlw==ia);
        ib_real = find(kListAlw==ib);
        [F0,P0] = and(P(ia_real:ib_real)');
        F = [F;F0,P_alw(i)==P0];
    end
    
end


function [F,P_ev] = eventually(P, a,b,kList,kMax)
    F = [];
    k = size(kList,2);
    P_ev = sdpvar(1,k);
    kListEv = unique(cell2mat(arrayfun(@(k) {min(kMax,k + a) : min(kMax,k + b)}, kList)));   
    for i = 1:k
        [ia, ib] = getIndices(kList(i),a,b,kMax);
        ia_real = find(kListEv==ia);
        ib_real = find(kListEv==ib);
        [F0,P0] = or(P(ia_real:ib_real)');
        F = [F;F0,P_ev(i)==P0];
    end
    
end

function [F,P_until] = until(Pp,Pq,a,b,k)
    
    %PhiU[a,b]Psi = G[0,a]Phi /\ F[a,b]Psi /\ F{a}(PhiUPsi)

    %G[0,a]Phi
    [F1,P1] = always(Pp, 0,a, k);
    
    %F[a,b]Psi
    [F2,P2] = eventually(Pq, a,b, k);
    
    %(PhiUPsi)
    [F_unt,P_unt] = until_untimed(Pp,Pq,k);
    
    %F{a}(PhiUPsi)
    [F3,P3] = eventually(P_unt, a,a, k);
    
    %and
    [F_and,P_until] = and([P1,P2,P3]);
    
    F = [F1,F2,F_unt,F3,F_and];
end



function [F,P_until] = until_untimed(Pp,Pq,k)
    %(this is the untimed until)
    F = [];
    
    % Auxiliary until
    P_until_aux = sdpvar(k,1);
    
    % i=1,...,k-1
    [F0,P0] = and([Pp(1:k-1),P_until_aux(2:k)]);
    [F1,P1] = or([Pq(1:k-1),P0(1:k-1)]);
    F = [F,F0,F1, P_until_aux(1:k-1) == P1];

    % i=k
    F = [F, P_until_aux(k) == Pq(k)];       

    % Main until
    P_until = sdpvar(k,1);   
    % i=1,...,k-1
    [F0,P0] = and([Pp(1:k-1),P_until(2:k)]);  
    [F1,P1] = or([Pq(1:k-1),P0(1:k-1)]);
    F = [F,F0,F1, P_until(1:k-1) == P1];

    % i=k
    [F0,P0] = and(P_until_aux(2:k)); % use auxiliary until here
    [F1,P1] = or(P0');
    [F2,P2] = and([Pp(k),P1]);
    [F3,P3] = or([Pq(k),P2]);
    F = [F, F0,F1,F2,F3, P_until(k) == P3]; 
end


function [ia, ib] = getIndices(i,a,b,k)
    ia = min(k,i+a);
    ib = min(k,i+b);
end
