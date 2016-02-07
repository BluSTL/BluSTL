classdef quad_system <STLC_lti
    
    properties 
        
        env;
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
              
            numObs = 1;
            
            Bw = zeros(size(Bu,1),numObs); 
            % complete Bw with 0s column for the number of obstacles
              
            C = [1 zeros(1, 9); 
             0 1 zeros(1, 8);
             0 0 1 zeros(1, 7)];
           
            % calls super class constructor
            QS = QS@STLC_lti(A,Bu,Bw,C,[],[]);
            
            QS.xlabel{1} = 'x';
            QS.xlabel{2} = 'y';
            QS.xlabel{3} = 'z';
            QS.xlabel{9} = 'vel';
            QS.xlabel{10} = 'omega';
            for i=1:numObs
                QS.wlabel{i} = strcat('obs',int2str(i));
            end
            
            % env
          %  ex=3;
          %  QS.env = envLTL(ex);
                
            % Init controller stuff
            
            
        end
    end
    
    methods
        function QS = init_control(QS)
            %% Controller Initialisation
            % Time
            QS.time = 0:.1:100; % time for the dynamics
            QS.ts=.5; % sampling time for controller
            QS.L=8;  % horizon (# of steps)
            QS.nb_stages=2; % repeats time

            QS.max_react_iter=100;
            QS.min_rob = 0.0;
            QS.lambda_rho = 10;
            QS.bigM = 1e6;
            
            % Input constraints
            QS.u_ub(:)=1;
            QS.u_lb(:)= -1;
            
            % Initial state
            QS.x0 = [0.1; 0.1; 0.1; zeros(7,1)];
            
            % STL formula
             QS.stl_list = {'alw (x(t)<5 and x(t)>0)','alw (y(t)<5 and y(t)>0)','alw (z(t)<5 and z(t)>0)'};     
             QS.stl_list{end+1} = 'alw ( (x(t)+y(t)+z(t) < 1)  => (ev_[0, 5] (x(t)+y(t)+z(t) > 14)))';            
             QS.stl_list{end+1} = 'alw ( (x(t)+y(t)+z(t) > 14)  => (ev_[0, 5] (x(t)+y(t)+z(t) < 1)))';

%             QS.stl_list{end+1} = 'alw ( (x(t)+y(t)+z(t) > 15)  => (ev_[0, 8] (x(t)+y(t)+z(t) < 3)))';
             %QS.stl_list{end+1} = 'alw ((y(t) < 1)  => (ev_[0, 5] (y(t) > 5)))'
             %QS.stl_list{end+1} = 'alw ((y(t) > 5)  => (ev_[0, 5] (y(t) < 1)))';

             
%             QS.stl_list{end+1} = 'alw ((obs1(t)<0) or ((x(t)<2) or (y(t)<2)))';
             
            
           % QS = getSpecs(QS);
            
            %% Plotting
            QS.plot_x = [1 2 3];
            QS.plot_y=[];
            
            %% Running stuff
            fprintf('Computing controller...\n');
            
            tic
%            QS.controller = get_controller(QS, 'interval');
            QS.controller = get_controller(QS, 'robust');
            toc            
          
        end
        
        function QS = update_plot(QS)
            QS = quad_plot(QS);
        end
        
        function Wn = sensing(QS)
                Wn = quad_sensing(QS);
        end
        
    end
end
        
        % specialized update_plot for quad_system class
        function QS = simple_plot(QS)
            
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
                  
                QS.h.hf2 = figure;
    
                lg = {'x', 'y', 'z'};
                for iY = 1:3
                    subplot(3,1,iY);
                    hold on;
                    QS.h.Ypast(iY) = plot(QS.system_data.time, QS.system_data.Y(iY,:));
                    QS.h.Ymodel(iY) = plot(QS.model_data.time(:,1:end-1),QS.model_data.Y(iY,:), '--r');
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
        
        function Wn = quad_sensing(QS)
            
            x_now = QS.x0;
            nw = QS.nw;
            Wn = zeros(1,2*QS.L);
            
            for i=1:nw % for each obstacle
                if x_now(1) > 1 % TODO: define sensing criteria for each obstacle
                    Wn(iwx,:) = ones(1,2*QS.L);
                end
            end

        end


    
    
