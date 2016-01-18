function IS = int_plot(IS)
        % quad_plot updates the plots of a quad_system at runtime
        %
        % Input: IS, the quad_system instance
        % Output: IS with plots modified

        system_style= {'LineWidth',2};
        model_style = {'--g'};
        axis_style = {'Fontsize',12}; 

        if isempty(IS.h)

            IS.h.hf = figure;
            nb_plots = numel(IS.plot_x)+numel(IS.plot_y)+numel(IS.plot_u)+numel(IS.plot_w);
            cur_plot = 1;


            %% Create the environment
            ex = 6;
            env = envLTL(ex);
            obstacles = env.work.unsafe;
            IS.h.hf = plotEnv(env, obstacles);

            hold on;
            zlabel('z (m)');
            
            IS.h.Xpred = plot3(IS.model_data.Y(1,:),IS.model_data.Y(2,:), IS.model_data.Y(3,:), ...
                        'og', 'markersize', 6, 'markerfacecolor', 'g');

            hold on;
            IS.h.Xpast = plot3(IS.system_data.Y(1,:)', IS.system_data.Y(2,:)', IS.system_data.Y(3,:)', ...
                        'ob', 'markersize', 6, 'markerfacecolor', 'b');
                    
            hbutton=uicontrol(IS.h.hf,'style','pushbutton',...
                                  'string','Stop',...
                                  'callback','Stop()'...
                              );        
        else

            set(IS.h.Xpast,'XData', IS.system_data.Y(1,end), 'YData', IS.system_data.Y(2,end), 'ZData', IS.system_data.Y(3,end));
            set(IS.h.Xpred,'XData', IS.model_data.Y(1,end-IS.L:end), 'YData', IS.model_data.Y(2,end-IS.L:end), 'ZData', IS.model_data.Y(3,end-IS.L:end));


        end
    end