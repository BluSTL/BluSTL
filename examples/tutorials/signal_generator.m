classdef signal_generator < STLC_lti
   
    methods
        function    SG= signal_generator()
            A =[];
            B =[];
            C =[];
            D = eye(2);
            SG = SG@STLC_lti(A,B,C,D);
        end
    end
    
end