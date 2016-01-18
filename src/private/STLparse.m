function phi = STLparse(phi,varargin)
%STLparse fills the field type, phi, phi1 and phi2. Note: this parser is not optimized/efficient at all, don't try it with big formulas
%
% Synopsis: phi = STLparse(phi, phi_str)
%
%      OR : phi = STLparse(phi, unary_op, phi0)
%      OR : phi = STLparse(phi, unary_op2, interv, phi2)
%      OR : phi = STLparse(phi, binary_op, phi1, phi2)
%      OR : phi = STLparse(phi, 'until', phi1, interv, phi2)
%      OR : phi = STLparse(phi, 'andn', [phi1, phi2, ..., phin])
%
% Inputs:
%  - phi       : is the STLformula to create
%  - phi_str   : a string describing the formula. This string must follow
%                the grammar described in the STLformula documentation
%  - phi0      : an STL Formula
%  - phi1      : an STL Formula
%  - phi2      : an STL Formula
%  - phin      : an STL Formula
%  - unary_op  : is either 'not', 'ev', 'alw', 'eventually' or 'always'
%  - unary_op2 : is either 'ev', 'alw', 'eventually' or 'always'
%  - binary_op : is either 'or', 'and' or '=>'
%  - interv    : is an interval
%
% Output:
%  - phi : a STL Formula structure
%

switch(numel(varargin))
    
    case 1 % Here, the formula is defined by a string. We parse this string.
        if ~ischar(varargin{1})
            error('STLformula:STLparse','Invalid formula');
        end
        
        % deals with true and false
        
        st = regexprep(varargin{1}, 'true', 'inf>0');
        st = regexprep(st, '^\s*', '');
        st = regexprep(st, 'false', 'inf<0');
        
        % aliases
        st = regexprep(st, 'always', 'alw');
        st = regexprep(st, 'eventually', 'ev');
        
        st = regexprep(st,'<=','<');
        st = regexprep(st,'>=','>');
               
        % test or
        [success, st1, st2] = parenthesisly_balanced_split(st, '\<or\>');
        if success
            phi1 = STLformula([phi.id '1__'],st1);
            phi2 = STLformula([phi.id '2__'],st2);
            phi = STLparse(phi,'or', phi1, phi2);
            return
        end
        
        % test implies
        [success, st1, st2] = parenthesisly_balanced_split(st, '\<=>\>');
        if success
            phi1 = STLformula([phi.id '1__'],st1);
            phi2 = STLformula([phi.id '2__'],st2);
            phi = STLparse(phi,'=>', phi1, phi2);
            return
        end
        
        % test and
        [success, st1, st2] = parenthesisly_balanced_split(st, '\<and\>');
        if success
            phi1 = STLformula([phi.id '1__'],st1);
            phi2 = STLformula([phi.id '2__'],st2);
            phi = STLparse(phi,'and', phi1, phi2);
            return
        end
        
        % test until
        [success, st1, st2] = parenthesisly_balanced_split(st, '\<until\>');
        interval = '[0 inf]';
        if success
            phi1 = STLformula([phi.id '1__'],st1);
            phi2 = STLformula([phi.id '2__'],st2);
            phi = STLparse(phi,'until', phi1, interval, phi2);
            return
        end
        
        % test until_[ti,tf]
        [success, st1, st2, interval] = parenthesisly_balanced_split(st, '\<until_\[(.+?)\]\>');
        if success
            phi1 = STLformula([phi.id '1__'],st1);
            phi2 = STLformula([phi.id '2__'],st2);
            phi = STLparse(phi,'until', phi1, interval, phi2);
            return
        end
        
        % test eventually
        [success, st1, st2] = parenthesisly_balanced_split(st, '\<ev\>');
        if (success&&isempty(st1))
            phi1 = STLformula([phi.id '1__'],st2);
            phi = STLparse(phi, 'ev', phi1);
            return
        end
        
        % test eventually_[ti,tf]
        [success, st1, st2, interval] = parenthesisly_balanced_split(st, '\<ev_\[(.+?)\]\>');
        if success&&isempty(st1)
            phi1 = STLformula([phi.id '1__'],st2);
            phi = STLparse(phi,'ev',interval,phi1);
            return
        end

        
        % test always
        [success, st1, st2] = parenthesisly_balanced_split(st, '\<alw\>');
        if success&&isempty(st1)
            phi1 = STLformula([phi.id '1__'],st2);
            phi = STLparse(phi, 'alw', phi1);
            return
        end
        
        % test alw_[ti,tf]
        [success, st1, st2, interval] = parenthesisly_balanced_split(st, '\<alw_\[(.+?)\]\>');
        if success&&isempty(st1)
            phi1 = STLformula([phi.id '1__'],st2);
            phi = STLparse(phi,'alw',interval,phi1);
            return
        end

        % test not
        [success, st1, st2] = parenthesisly_balanced_split(st, '\<not\>');
        if success&&isempty(st1)
            phi1 = STLformula([phi.id '1__'],st2);
            phi = STLparse(phi, 'not', phi1);
            return
        end
        
        % test predicate
        
        [success, st1, st2] = parenthesisly_balanced_split(st, '<');
        if success
            phi.type='predicate';
            phi.st = [ st1 '<' st2 ];
            return
        end
        
        [success, st1, st2] = parenthesisly_balanced_split(st, '>');
        if success
            phi.type = 'predicate';
            phi.st = [ st1 '>' st2 ];
            return
        end
        
        % Last possibility, the formula already exists
        
        try
            id = phi.id;
            phi = struct(evalin('base', st));
            phi.id = id;
        catch
            error('STLformula:STLparse',['Unknown or ill-formed predicate or subformula: ' st]);
        end
        
    case 2
        switch(varargin{1})
            case 'not'
                phi.type = 'not';
                phi.phi = varargin{2};
                
            case 'ev'
                phi.type = 'eventually';
                phi.phi = varargin{2};
                phi.interval = '[0 inf]';
                
            case 'alw'
                phi.type = 'always' ;
                phi.phi = varargin{2};
                phi.interval = '[0 inf]';
                
            otherwise
                phi.st = varargin{1};
                phi.type = 'predicate';
        end
        
    case 3
        switch(varargin{1})
            case 'or'
                phi.type = 'or' ;
                phi.phi1 = varargin{2};
                phi.phi2 = varargin{3};
                
            case 'and'
                phi.type = 'and';
                phi.phi1 = varargin{2};
                phi.phi2 = varargin{3};
                
            case '=>'
                phi.type = '=>';
                phi.phi1 = varargin{2};
                phi.phi2 = varargin{3};
                
            case 'ev'
                phi.type = 'eventually' ;
                phi.interval = varargin{2};
                phi.phi = varargin{3};
                
            case 'alw'
                phi.type = 'always' ;
                phi.interval = varargin{2};
                phi.phi = varargin{3};
        end
        
    case 4
        switch(varargin{1})
            case 'until'
                phi.type = 'until' ;
                phi.interval =  varargin{3};
                phi.phi1 = varargin{2};
                phi.phi2 = varargin{4};
        end
    otherwise
        error('STLformula:STLparse','Too many arguments.')
end

end

function [success, st1, st2, interval] = parenthesisly_balanced_split(st, op, int)

% split st into st1 op st2 where st1 and st2 are parenthesisly balanced
success = 0;
st1 = '';
st2 = '';
interval ='';

[start_idx, end_idx, ~, ~, tokens] = regexp(st,op);

for i = 1:numel(start_idx)
    
    % checks left hand side
    st1 = st(1:start_idx(i)-1);
    st2 = st(end_idx(i)+1:end);
    
    [success, diag, st1, st2] = checks_parenthesis_balance(st1,st2);
    if success==-1
        error(['STLparse: expression ' st ':' diag]);
    elseif success==1
      if nargout == 4
        interval= ['[' tokens{i}{1} ']'];
      end
      return
    end
end

end

function [success, diag, st1, st2] = checks_parenthesis_balance(st1,st2)

success=0;
diag = '';

% finds parenthesis
idx_left_par1 = regexp(st1,'(');
idx_right_par1 = regexp(st1,')');

nb_left_par1 = numel(idx_left_par1);
nb_right_par1 = numel(idx_right_par1);

idx_left_par2 = regexp(st2,'(');
idx_right_par2 = regexp(st2,')');

nb_left_par2 = numel(idx_left_par2);
nb_right_par2 = numel(idx_right_par2);

% first sanity check: equal total number of ( and )

diff_par = (nb_left_par1+nb_left_par2) - (nb_right_par1+nb_right_par2);
if (diff_par>0)
    diag=sprintf('Too many (%d) opening parenthesis in expr', diff_par);
    success=-1;
    return;
elseif (diff_par<0)
    diag=sprintf('Too many (%d) closing parenthesis in expr', -diff_par);
    success=-1; 
    return;
end

% checks parenthesis for st1

% first check/remove enclosing parenthesis: we should have (*par_exp where par exp is balanced
% so we check the difference in the number of left and right, if more right, then problem

diff1 = nb_left_par1- nb_right_par1; % from previous check, diff2 = -diff1

if (diff1 ~=0)
    if (diff1<0)
        success=0;
        return;
    else % alright, so diff1>0 should be the number of enclosing parenthesis
        % we remove them
        
        % checks if there is nothing but blanks before enclosing par. 
        pre_st1 = st1(1:idx_left_par1(diff1));  
        if ~isempty(regexp(pre_st1, '[^\(\s]')) 
            success= 0;
            return;
        end
        
        % checks if there is nothing but blanks after enclosing par. 
        post_st2 = st2(idx_right_par2(end-diff1+1):end);  
        if ~isempty(regexp(post_st2, '[^\)\s]')) 
            success= 0;
            return;
        end

        st1 = st1(1+idx_left_par1(diff1):end);
        idx_left_par1 = idx_left_par1(1+diff1:end);
        
        st2 = st2(1:idx_right_par2(end-diff1+1)-1);
        idx_right_par2 = idx_right_par2(1:end-diff1);
      
    end
end    
    % At this point, no enclosing parenthesis any more, st1 and st2 should be balanced
    success = check_par(idx_left_par1, idx_right_par1) && check_par(idx_left_par2, idx_right_par2);
end


function success = check_par(idx_left_par, idx_right_par)

% idx_left_par and idx_right_par have same number of elements

assert(numel(idx_left_par) == numel(idx_right_par));
counter = 0;

%read stuff; +1 if left par, -1 if right. Whenever counter<0 exit with success==0
lcount =1;
rcount =1;
nb_par = numel(idx_left_par);

while (1)

    % no more left or right par.
    if (lcount > nb_par) && (rcount>nb_par)
        break;
    end
    
    if (lcount > nb_par) % no more left, add one right
        counter = counter-1;
        rcount = rcount+1;
    elseif (rcount > nb_par) % no more right, add one left
        counter=counter+1;
        lcount=lcount+1;
    else % still some left and right parenthesis
        
        next_lp = idx_left_par(lcount);
        next_rp = idx_right_par(rcount);
        if (next_lp<next_rp) % next par is left, add left
            counter=counter+1;
            lcount=lcount+1;
        else % next par is right, add right
            counter=counter-1;
            rcount=rcount+1;
        end
    end
    
    % if count went neg, no success
    if counter<0
        break;
    end
    
end

success = (counter==0); % meaning all left par have been consumed by right par


end


