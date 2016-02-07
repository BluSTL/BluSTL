classdef integrator < STLC_lti
    % this class implements a "signal generator". It has no dynamics and no
    % disturbance, so that the control synthesis problem is the same as
    % finding an input signal that satisfies a given STL formula
    properties
        n_int
    end
    
    methods
        function  IS = integrator(n_int)
            
            nx = 3*n_int;   % num states
            nu = 3;         % num inputs
            nw = 3;

            A = [diag(ones(n_int-1,1),1), zeros(n_int), zeros(n_int);
                  zeros(n_int), diag(ones(n_int-1,1),1), zeros(n_int);
                  zeros(n_int), zeros(n_int), diag(ones(n_int-1,1),1)];

            Bu =zeros(nx,nu);
            Bu(n_int,1) = 1;
            Bu(2*n_int,2) = 1;
            Bu(3*n_int,3) = 1;
            
            numObs = 1;
            
            Bw = [zeros(nx,nw),zeros(nx,numObs)];
            Bw(n_int,1) = 1;
            Bw(2*n_int,2) = 1;
            Bw(3*n_int,3) = 1;
            
     
            C = zeros(3,nx);
            C(1,1) = 1;
            C(2,n_int+1) = 1;
            C(3,2*n_int+1) = 1;
            Du = [];
            Dw = [];
            
            
            IS = IS@STLC_lti(A,Bu,Bw,C,Du,Dw);
            
            IS.n_int = n_int;
            
            IS.ylabel{1} = 'x';
            IS.ylabel{2} = 'y';
            IS.ylabel{3} = 'z';
            
            for i=1:numObs
                IS.wlabel{nw+i} = strcat('obs',int2str(i));
            end
            
            % env
            %  ex=3;
            %  IS.env = envLTL(ex);
                
            % Init controller stuff
        
        end
        
        function IS = init_control(IS)
            %% Controller Initialisation
            % Time
            IS.ts=1; % sampling time for controller
            IS.time = 0:1:100; % time for the dynamics
            IS.L=10;  % horizon (# of steps)
            IS.nb_stages=1; % repeats time

            IS.max_react_iter=100;
            IS.min_rob = 0.01;
            IS.lambda_rho = 1000;
            
            % Input constraints
            IS.u_ub(:)=1;
            IS.u_lb(:)=-1;
            
            % Initial state
            x_init = zeros(IS.nx, 1);
            x_init(1) = 0.5;
            x_init(IS.n_int + 1) = 0.5;
            x_init(2*IS.n_int + 1) = 0.25;
            IS.x0 = x_init;
            
            %IS.Wref = [0.001*sin(IS.time); 0.001*cos(IS.time); zeros(1,numel(IS.time)); ones(1,numel(IS.time))];
            %IS.Wref = [zeros(3,numel(IS.time)); ones(1,numel(IS.time))];
            IS.Wref = [zeros(3,numel(IS.time)); zeros(1,20),ones(1,20),zeros(1,20),ones(1,numel(IS.time)-60)];
            
            
            %% STL formula
            
    
             
             IS.stl_list = {'alw (x(t)<10 and x(t)>0)','alw (y(t)<10 and y(t)>0)','alw (z(t)<10 and z(t)>0)'};
             
             
             IS.stl_list{end+1} = '( ( (x(t)<1)  =>  ev_[0, 10] (x(t)>8)) and ((x(t)>8)  =>  ev_[0, 10] (x(t)<1)))';
             IS.stl_list{end+1} = '( ( (y(t)<1)  =>  ev_[0, 10] (y(t)>8)) and ((y(t)>8)  =>  ev_[0, 10] (y(t)<1)))';
             IS.stl_list{end+1} = '( ( (z(t)<1)  =>  ev_[0, 10] (z(t)>8)) and ((z(t)>8)  =>  ev_[0, 10] (z(t)<1)))';
             
             
             IS.stl_list{end+1} = 'alw ((obs1(t)<0) or ((x(t)<2) or (y(t)<2)) or ((x(t)>4) or (y(t)>4)))';
             
           
            
            %IS = getSpecs(IS);
            
            %% Plotting
            IS.plot_x=[];
            IS.plot_u=[];
            IS.plot_y=[2];
            IS.plot_w=[2];
            
            %% Running stuff
            fprintf('Computing controller...\n');
            
            tic
            IS.controller = get_controller(IS, 'robust');
            toc            
          
        end
        
       function IS = update_plot(IS)
            IS = int_plot(IS);
       end
        
        function Wn = sensing(IS)
                Wn = int_sensing(IS);
        end
        
    end
end
        
        % specialized update_plot for quad_system class
        function IS = simple_plot(IS)
            
            if isempty(IS.h)
                %obstacles = IS.env.work.unsafe;
                %IS.h.hf = plotEnv(IS.env, obstacles);
                IS.h.hf = figure;
                grid on;
                hold on;
               
                axis([0 10 0 10 0 10]);
                zlabel('z (m)');
                IS.h.Ypast3d = plot3(IS.system_data.Y(1,:)', IS.system_data.Y(2,:)', IS.system_data.Y(3,:)', ...
                'ob:','markersize', 6, 'markerfacecolor', 'b');
                IS.h.Ymodel3d = plot3(IS.model_data.Y(1,:),IS.model_data.Y(2,:), IS.model_data.Y(3,:), ...
                'ok:','markersize', 6, 'markerfacecolor', 'k');
  
                IS.h.hbutton=uicontrol(IS.h.hf,'style','pushbutton',...
                          'string','Stop',...
                          'callback','Stop()'...
                      );        
                  
                IS.h.hf2 = figure;
    
                lg = {'x', 'y', 'z'};
                for iY = 1:3
                    subplot(3,1,iY);
                    hold on;
                    IS.h.Ypast(iY) = plot(IS.system_data.time, IS.system_data.Y(iY,:));
                    IS.h.Ymodel(iY) = plot(IS.model_data.time(:,1:end-1),IS.model_data.Y(iY,:), '--r');
                    ylabel(lg{iY});
                end
                
            else
                
                set(IS.h.Ymodel3d,'XData', IS.model_data.Y(1,:), 'YData', IS.model_data.Y(2,:), 'ZData', IS.model_data.Y(3,:));
                set(IS.h.Ypast3d,'XData', IS.system_data.Y(1,:), 'YData', IS.system_data.Y(2,:), 'ZData', IS.system_data.Y(3,:));
                for iY = 1:3
                    set(IS.h.Ypast(iY),'XData', IS.system_data.time, 'YData', IS.system_data.Y(iY,:));
                    set(IS.h.Ymodel(iY),'XData', IS.model_data.time, 'YData', IS.model_data.Y(iY,:));
                end
                
            end
        end
        
        function Wn = int_sensing(IS)
            
            x_now = IS.x0;
            nw = IS.nw;
            nu = IS.nu;
            Wn = zeros(nw,2*IS.L);
            
            for i=nu+1:nw % for each obstacle
                if x_now(1) > 1 % TODO: define sensing criteria for each obstacle
                    Wn(iwx,:) = ones(1,2*IS.L);
                end
            end

        end
 