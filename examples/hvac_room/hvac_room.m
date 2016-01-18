classdef hvac_room <STLC_lti
    
    methods
        function HR = hvac_room()
            load hvac_room_data  % note: this file is created by Init_RoomHVAC
            %% Create the system
            HR = HR@STLC_lti(A,Bu,Bw);
            
            %% Naming stuff
            
            HR.xlabel{1} = 'Twall1';
            HR.xlabel{2} = 'Twall2';
            HR.xlabel{3} = 'Twall3';
            HR.xlabel{4} = 'Twall4';
            HR.xlabel{5} = 'Troom';
            HR.wlabel{1} = 'Tdis8';
            HR.wlabel{2} = 'T7';
            HR.wlabel{3} = 'Tout';
            HR.wlabel{4} = 'T10';
            HR.wlabel{5} = 'Qsun';
            HR.wlabel{6} = 'Tcomf_low';
            HR.wlabel{7} = 'occ';
            
            %   HR = init_control(HR);
        end
    end
    
    methods
        function HR = init_control(HR,L,epsi, lambda_rho)
            load hvac_room_data
            
            if epsi==0
                HR.is_det = 1;
            else
                HR.is_det = 0;
            end
            
            %% Controller Initialisation
            % Time
            HR.time = 0:10:1439; % time for the dynamics
            HR.ts=30; % sampling time for controller
            HR.L=L;  % horizon (# of steps)
            HR.nb_stages=3; % repeats time
            
            HR.max_react_iter=100;
            HR.min_rob = .1;
            HR.lambda_rho = lambda_rho;
            HR.bigM = 1000;
            
            % Input constraints
            HR.u_lb=0;
            HR.u_ub=65;
            HR.u_delta=Inf;
            
            % Disturbance signal
            HR.Wref = Wref(:,HR.time+1);
            meanw = mean(Wref(1:5,:)');
            
            epsi = epsi*meanw/100; % 10% possible deviation from Wref
            
            HR.w_lb(1:5) = -epsi;
            HR.w_ub(1:5) = epsi;
            HR.solver_options = sdpsettings('solver','gurobi','verbose',1, 'cachesolvers',1);
            
            %HR.w_ub(7) = 2;
            
            % Initial state
            HR.x0 = X0;
            
            %% STL formula
            HR.stl_list{1} = 'alw_[0, Inf] ((100*occ(t) > 0) => (Troom(t) > Tcomf_low(t)))';
            %          HR.stl_list{2} = 'alw_[0, Inf] ( X(5,t)> 67)';
            
            %% Plotting
            HR.plot_x = [5];
            HR.plot_w = [4 7];
            
            %% Running stuff
            fprintf('Computing controller...\n');
            tic
            HR.controller = get_controller(HR);
            toc
            if ~HR.is_det
                fprintf('Computing adversary...\n');
                tic
                HR.adversary = get_adversary(HR);
                toc
            end
            
        end
        
        function [Y,T,X] = system_step(Sys, u0, t, x0, w0)
            U = [u0; w0];
            [Y,T,X] = lsim(Sys.sys, U',t-t(1),x0);
        end
        
        
        function HR = update_plot(HR)
            
            if isempty(HR.h)
                
                time = HR.time;
                Wref = HR.Wref;
                
                XLim = [0 HR.time(end)/60];
                HR.h.hf = figure;
                
                % Temperature
                subplot(5,1,1:3);
                hold on; grid on;
                
                % Troom
                HR.h.Tpast = plot(HR.system_data.time/60, HR.system_data.X(5,:), 'LineWidth',2);
                HR.h.Tmodel = plot(HR.model_data.time/60,HR.model_data.X(5,:), '--g','LineWidth',2);
                
                % Tout
                HR.h.Toutpast = plot(HR.system_data.time/60, HR.system_data.W(3,:), '-m', 'LineWidth',2);
                
                % Comfort region
                plot(time/60, Wref(6,:), '-k','LineWidth',2)
                legend('Room Temperature', 'Model prediction', 'Outside Temperature','Comfort Region')
                plot(time/60, 2*70-Wref(6,:), '-k','LineWidth',2)
                
                if ~HR.is_det
                    % Tout bounds
                    plot(time/60, Wref(3,:)+HR.w_lb(3), '-m','LineWidth',1)
                    plot(time/60, Wref(3,:), '--m','LineWidth',1);
                    plot(time/60, Wref(3,:)+HR.w_ub(3), '-m','LineWidth',1)
                end
                
                %  xlabel('Time (hours)')
                ylabel('Temperatures (°F)');
                set(gca, 'XLim',XLim , 'XTick', 0:3:time(end)/60, 'FontSize', 14 );
                
                % occupancy
                subplot(5,1,4);
                hold on;grid on;
                stairs(time(1:end)/60, Wref(7,1:end), '-k', 'LineWidth',2);
                legend('Occupancy');
                % xlabel('Time (hours)')
                set(gca, 'XLim', XLim, 'YLim', [-1.1 1.1], 'XTick', 0:3:time(end)/60, ...
                    'FontSize', 14, 'YTick', [-1 1], 'YTickLabel', {'empty' 'occupied'});
                
                
                subplot(5,1,5);
                hold on;grid on;
                HR.h.Upast = stairs(HR.system_data.time/60, HR.system_data.U(1,:),'LineWidth',3);
                HR.h.Umodel = stairs(HR.model_data.time(1:end-1)/60,HR.model_data.U(1,:), '--gr','LineWidth',2);
                xlabel('Time (hours)')
                ylabel('u');
                
                legend('Input Air Flow (ft^3/min)');
                set(gca, 'XLim', XLim, 'XTick', 0:3:time(end)/60, 'FontSize', 14);
                
            else
                set(HR.h.Tpast, 'Xdata', HR.system_data.time/60, 'Ydata',HR.system_data.X(5,:));
                set(HR.h.Toutpast, 'Xdata', HR.system_data.time(1:2:end)/60, 'Ydata',HR.system_data.W(3,1:2:end));
                set(HR.h.Tmodel, 'Xdata', HR.model_data.time/60,'Ydata', HR.model_data.X(5,:));
                set(HR.h.Upast, 'Xdata', HR.system_data.time/60,'Ydata',  HR.system_data.U(1,:));
                set(HR.h.Umodel, 'Xdata', HR.model_data.time(1:end-1)/60,'Ydata', HR.model_data.U(1,:));
                
            end
            
            
        end
        
    end
    
end


