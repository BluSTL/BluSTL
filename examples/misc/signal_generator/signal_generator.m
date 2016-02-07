classdef signal_generator < STLC_lti
    % this class implements a "signal generator". It has no dynamics and no
    % disturbance, so that the control synthesis problem is the same as
    % finding an input signal that satisfies a given STL formula
    
    methods
        function  SG = signal_generator(nu, nw)
            A =[];
            B =[];
            C =[];
            Du = eye(nu);
            Dw = eye(nw);
            SG = SG@STLC_lti(A,B,[],C,Du,Dw);
            SG = init_control(SG);
        end
        
        function SG = init_control(SG)
            SG.nrm = 1;
            SG.lambda_rho = 1000;
            SG.lambda_t1 = 100;
        end
        
    end
    
end