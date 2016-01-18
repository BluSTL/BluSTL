clear all;
load('hvac_room_model_data.mat');

M8=x(1);
Mw1=x(2);
Mw2=x(3);
Mw3=x(4);
Mw4=x(5);
Rvalw=x(6);
Rvalgl=x(7);
Rvalin=x(8);
Rvalout=x(9);

tau=0.5;                                % tau=0.5 is the transmittance of glass    
alpha= 0.2;                             % alpha=0.2 is the absoprtivity of wall 2  
Qtrans= (H*L2*beta2) * tau * Qsun;      % WE Assume Qdot only radiates on wall 2
Qabs=(H*L2*(1-beta2)) * alpha * Qsun;   % WE Assume Qdot only radiates on wall 2


Rin1=Rvalin/(H*L1*(1-beta1));     Rin3=Rvalin/(H*L1*(1-beta3));         %R for inside air on wall 1 & 3
Rin2=Rvalin/(H*L2*(1-beta2));     Rin4=Rvalin/(H*L2*(1-beta4));         %R for inside air on wall 2 & 4
Rw1=Rvalw/(H*L1*(1-beta1));       Rw3=Rvalw/(H*L1*(1-beta3));           %R for wall # 1 & 3
Rw2=Rvalw/(H*L2*(1-beta2));       Rw4=Rvalw/(H*L2*(1-beta4));           %R for wall # 2 & 4
Rout1=Rvalout/(H*L1*(1-beta1));   Rout3=Rvalout/(H*L1*(1-beta3));       %R for ouside air on wall 1 & 3
Rout2=Rvalout/(H*L2*(1-beta2));   Rout4=Rvalout/(H*L2*(1-beta4));       %R for outside air on wall 2 & 4
Rwin2tot=(Rvalin+Rvalgl+Rvalout)/(H*L2*beta2);                          %R total for windows section of wall 2
Rwin3tot=(Rvalin+Rvalgl+Rvalout)/(H*L1*beta3);
         
%% Create the linearized system model

LT=1084;                            % Time at which we linearize the system
x0=TSim8(LT,1:5)';                  % state near which we are seeking an equilibrium point
u0=[mdot8(LT) Tdis8(LT) T7(LT) Tout(LT) T10(LT) Qsun(LT)]';     % input near which we are seeking an equilibrium point
[xe,ue,ye,dxe,options] = trim('hvac_room_model',x0,u0);     % trim gives back the equilibrium point near the given points

[A,B,C,D] = linmod('hvac_room_model',xe,ue);

sys = ss(A,B,C,D);
 
%% Disturbance signal
Wref = [Tdis8'; T7'; Tout'; T10'; Qsun'];

%% AUXILIARY SIGNALS
% Init comfort zone
Tave=21;    % average confort temperature
DTocc=1;    % tolerance with occupants
DTunocc=4;  % tolerance without occupants
N = 1440;
Delta = 2;  %  transition from occupied to not occupied 

[lowTC, upTC] = get_temp_bnd(Tave, DTocc, DTunocc, Delta, N);
lowTC= lowTC';

lowTF = lowTC*1.8+32; % lowTC is the lower limit of temperature in °C, lowT is in °F
occ = [-1*ones(1,13*30),... % 6:30  day 1
    ones(1,4*60), ...    % 10:30
    -1*ones(1,3*60),...  % 13:30
    ones(1,4*60),...     % 17:30
    -1*ones(1,13*30)];   % 24:00

Wref = [Wref;lowTF;occ];

%% Initial state
X0=IC';      % ^oC

Bu = B(:,1);
Bw = [B(:,2:end) zeros(size(Bu,1),2)]; % complete Bw with 0s column for lowTF and occ

save('hvac_room_data', 'A', 'Bu', 'Bw', 'Aux', 'Wref','X0');