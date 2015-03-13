classdef grid_system < STLC_lti
    
    methods
        
        % constructor
        function GS = grid_system()
            % Area 1 constants
            Mx=132.6;
            Dx=0.0265;
            nuxy=1;
            Tx1=0.1; Tx3=0.1; Tx4=1;
            Kx1=1;
            Rx=0.05;
            Kix=0.03;  % tuning parameter of PI ctrl. % oscillatory at 0.3
            betax=Dx+(1/Rx);
            
            % Area 2 constants
            My=663.13;
            Dy=0.1325;
            nuyx=1;
            Ty1=0.2;  Ty3=0.3;  Ty4=0.1;  Ty5=0.5;
            Ky1=0.2;  Ky3=3;
            Ry=0.05;
            Kiy=0.02;  % tuning parameter of PI ctrl.
            betay=Dy+(1/Ry);
            
            % System Dynamics
            A=[  ...
                -Dx/Mx Kx1/Mx       0       0  -nuxy/Mx   0             0        0        0        0        0    nuxy/Mx   0;
                0   -1/Tx4    1/Tx4       0         0   0             0        0        0        0        0          0   0;
                0        0   -1/Tx3   1/Tx3         0   0             0        0        0        0        0          0   0;
                -1/(Rx*Tx1) 0        0  -1/Tx1         0  -Kix/Tx1       0        0        0        0        0          0   0;
                1        0        0       0         0   0             0        0        0        0        0          0   0;
                betax        0        0       0      nuxy   0             0        0        0        0        0      -nuxy   0;
                0        0        0       0   nuyx/My   0        -Dy/My   Ky3/My        0        0   Ky1/My   -nuyx/My   0;
                0        0        0       0         0   0             0   -1/Ty5    1/Ty5        0        0          0   0;
                0        0        0       0         0   0             0        0   -1/Ty4    1/Ty4        0          0   0;
                0        0        0       0         0   0             0        0        0   -1/Ty3    1/Ty3          0   0;
                0        0        0       0         0   0   -1/(Ry*Ty1)        0        0        0   -1/Ty1          0   -Kiy/Ty1;
                0        0        0       0         0   0             1        0        0        0        0          0   0;
                0        0        0       0     -nuyx   0         betay        0        0        0        0       nuyx   0];
            
            
            Bu=[1/Mx 0;  % for Manipulatable input
                0 0;
                0 0;
                0 0;
                0 0;
                0 0;
                0 1/My;
                0 0;
                0 0;
                0 0;
                0 0;
                0 0;
                0 0];
            
            Bw=[-1/Mx 0;  % for Disturbance input
                0 0;
                0 0;
                0 0;
                0 0;
                0 0;
                0 -1/My;
                0 0;
                0 0;
                0 0;
                0 0;
                0 0;
                0 0];
            
            % Y should be the ACE for x and y
            C= A([6 13],:);
            Du= [];
            Dw =[];
            
            % calls super class constructor
            GS = GS@STLC_lti(A,Bu,Bw,C, Du, Dw);
        end
                
        
        
    end
end