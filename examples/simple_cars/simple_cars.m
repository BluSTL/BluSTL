classdef simple_cars < STLC_lti
    
    methods
        function SC = simple_cars()
            
            A1 = [0, 1; 0, 0];
            B1 = [0; 1];
            
            A2 = [0, 1; 0, 0];
            B2 = [0; 1];
            
            IA = eye(2);
            OA = zeros(2);
            OAv = zeros(2,1);
            
            A = [A1 OA;
                OA A2];
            B = [B1 OAv;
                OAv B2];
            
            Bu = B(:,1);
            Bw = B(:,2);
            
            SC = SC@STLC_lti(A,Bu,Bw);
            
        end
        
        function obj = get_objective(Sys, X, Y, U, W, rho, wr)
            
            % Objective is speed close to 1
            obj = norm(X(2,1)-1,1);
            for k=2:2*Sys.L
                obj = obj + Sys.ts*(norm(X(2,k-1)-1,1));
            end
            
        end
        
        
    end
    
    
end