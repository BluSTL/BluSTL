function [inits] = getSpecs2( inits, dim, ex)

% Control constraints
inits.var.Hu = [eye(3); -eye(3)];
inits.var.Ku = 0.04*[9.935; 3.62; 3.62;  
           4.545; 3.62; 3.62];

      
inits.stl_list = {%'alw_[0,Inf] ((var.Hx*X(:,t) <= var.Kx))', ...
    %'alw_[0,Inf] ((var.Hu*U(:,t-1) <= var.Ku))', ...
    
%     'alw_[0,Inf] ((ev_[0,6] (X(1:2,t) > [8;8] and  X(1:2,t) < [9;9])))', ...
%     'alw_[0,Inf] ((ev_[0,6] (X(1:2,t) > [1;1] and  X(1:2,t) < [2;2])))'};
     
    'ev_[0,Inf] ((X(1:2,t) < [9;9]) and (X(1:2,t) > [8;8]))', ...
    'ev_[0,Inf] ((X(1:2,t) < [1;1]) and (X(1:2,t) > [0.5;0.5]))', ...
    'ev_[0,Inf] ((X(1:2,t) < [4;2]) and (X(1:2,t) > [3;1]))'};

    %'alw_[0,Inf] (ev_[0,6] (X(1:2,t) >= [1;1]))', ...
    %'alw_[0,Inf] (ev_[0,6] (X(1:2,t) <= [0.5;0.5]))'};%, ...
    
    
end

