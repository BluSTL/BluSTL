classdef quad_system < STLC_lti
    
    properties 
    
        env
    end
    
    
    methods
        
        % constructor
        function QS = quad_system()
            % Quadrotor linearized about hover (van der Burg 13)
            % x = (p,v, r,w),     u = (uf,ux,uy)
            % p = position (m),   v = velocity (m/s)
            % r = attitude (rad), w = angular velocity (rad/s)
            %
            % Model adapted from LTLOpt (http://www.cds.caltech.edu/~ewolff/ltlopt.html)
            %
            %
            % Input: 'dt' timestep (sec)
            %        'N' length of traj
            %        'dim' workspace dimension (2D or 3D)
            % Output: 'X' sdpvar for state
            %         'U' sdpvar for control
            %         'F' dynamic constraints
            %         'sys' system info


            dim=3;
            
            % Parameters (modified from van der Burg 13)
            g = 9.81;       % gravity (m/s^2)
            m = 1;          % mass (kg)
            arm = 0.25;       % length of rotor arm (m)
            J = 20*m*arm^2;   % moment of inertia (kg m^2) (est)

            tmp = [0 g; -g 0; 0 0];  
           

            % System Dynamics
            A = [zeros(3),   eye(3),     zeros(3,2), zeros(3,2); 
                  zeros(3),   zeros(3),   tmp,        zeros(3,2); 
                  zeros(2,3), zeros(2,3), zeros(2),   eye(2);
                  zeros(2,3), zeros(2,3), zeros(2),   zeros(2)];


            tmp = [0; 0; 1/m];
            Bu = [zeros(3,1) zeros(3,2); 
                  tmp        zeros(3,2); 
                  zeros(2,1) zeros(2,2); 
                  zeros(2,1) arm/J*eye(2)];
           
            % calls super class constructor
            QS = QS@STLC_lti(A,Bu);
            
            % env
            ex=3;
            QS.env = envLTL(ex);
                
            % Init controller stuff
            
            
        end
        
        function QSys = init_control(QSys)
            %% Controller Initialisation
            % Time
            QSys.time = 0:1:100; % time for the dynamics
            QSys.ts=2; % sampling time for controller
            QSys.L=10;  % horizon (# of steps)
            QSys.nb_stages=1; % repeats time

            QSys.max_react_iter=100;
            QSys.min_rob = 0.1;
            QSys.lambda_rho = 10;
            
            % Input constraints
            QSys.u_ub(:)=10;
            QSys.u_lb(:)= -10;
            
            % Initial state
            QSys.x0 = [0.1; 0.1; 0.1; zeros(7,1)];
            
            %% STL formula
%            QSys.stl_list{1} = 'alw_[0,Inf] ( abs(x1(t))<.1 and abs(x2(t))<.1 and abs(x3(t)-1)<.1 )';
            QSys.stl_list{1} = 'alw_[0, Inf] ( ( x3(t)<1  =>  ev_[0, 10] (x3(t)>10)) and ((x3(t)>10)  =>  ev_[0, 10] (x3(t)<2)))';
            QSys.stl_list{2} = 'alw_[0,Inf] ( x3(t)<11 and x3(t)>0)';

            %           QSys.stl_list{3} = 'ev_[0, 10] (  x1(t)>10 ) and ev_[10, 20] (x1(t)<1)';
%           QSys.stl_list{4} = 'alw_[0,Inf] ( x1(t)<11 and x1(t)>0)';
            
            %% Plotting
            QSys.plot_x = [1:3];
            
            %% Running stuff
            fprintf('Computing controller...\n');
            
            tic
            QSys.controller = get_controller(QSys);
            toc            
            
            %  fprintf('Running closed loop...')
            %  HR = run_open_loop_adv(HR, controller, adversary);
            %  HR = run_adversarial(HR, controller, adversary);
            %  fprintf('\ndone.\n');

        end

        
        % specialized update_plot for quad_system class
        function QS = update_plots(QS)
            
            if isempty(QS.h)
                %obstacles = QS.env.work.unsafe;
                %QS.h.hf = plotEnv(QS.env, obstacles);
                QS.h.hf = figure;
                grid on;
                hold on;
               
                axis([0 10 0 10 0 10]);
                zlabel('z (m)');
                QS.h.Ypast3d = plot3(QS.system_data.Y(1,:)', QS.system_data.Y(2,:)', QS.system_data.Y(3,:)', ...
                'ob:','markersize', 6, 'markerfacecolor', 'b');
                QS.h.Ymodel3d = plot3(QS.model_data.Y(1,:),QS.model_data.Y(2,:), QS.model_data.Y(3,:), ...
                'ok:','markersize', 6, 'markerfacecolor', 'k');
  
                QS.h.hbutton=uicontrol(QS.h.hf,'style','pushbutton',...
                          'string','Stop',...
                          'callback','Stop()'...
                      );        
                  
                QS.system_data.U(1,:)  
                
                QS.h.hf2 = figure;
    
                lg = {'x', 'y', 'z'};
                for iY = 1:3
                    subplot(3,1,iY);
                    hold on;
                    QS.h.Ypast(iY) = plot(QS.system_data.time, QS.system_data.Y(iY,:));
                    QS.h.Ymodel(iY) = plot(QS.model_data.time,QS.model_data.Y(iY,:), '--r');
                    ylabel(lg{iY});
                end
                
            else
                
                set(QS.h.Ymodel3d,'XData', QS.model_data.Y(1,:), 'YData', QS.model_data.Y(2,:), 'ZData', QS.model_data.Y(3,:));
                set(QS.h.Ypast3d,'XData', QS.system_data.Y(1,:), 'YData', QS.system_data.Y(2,:), 'ZData', QS.system_data.Y(3,:));
                for iY = 1:3
                    set(QS.h.Ypast(iY),'XData', QS.system_data.time, 'YData', QS.system_data.Y(iY,:));
                    set(QS.h.Ymodel(iY),'XData', QS.model_data.time, 'YData', QS.model_data.Y(iY,:));
                end
                
            end
            
        end
        
    end
end