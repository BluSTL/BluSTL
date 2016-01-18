function QS = quad_plot(QS)
        % quad_plot updates the plots of a quad_system at runtime
        %
        % Input: QS, the quad_system instance
        % Output: QS with plots modified

        system_style= {'LineWidth',2};
        model_style = {'--g'};
        axis_style = {'Fontsize',12}; 

        if isempty(QS.h)

            QS.h.hf = figure;
            nb_plots = numel(QS.plot_x)+numel(QS.plot_y)+numel(QS.plot_u)+numel(QS.plot_w);
            cur_plot = 1;


            %% Create the environment
            ex = 6;
            env = envLTL(ex);
            obstacles = env.work.unsafe;
            QS.h.hf = plotEnv(env, obstacles);

            hold on;
            zlabel('z (m)');
            QS.h.Xpast = plot3(QS.system_data.Y(1,:)', QS.system_data.Y(2,:)', QS.system_data.Y(3,:)', ...
                        'ob', 'markersize', 6, 'markerfacecolor', 'b');
            hold on;
            QS.h.Xpred = plot3(QS.model_data.Y(1,:),QS.model_data.Y(2,:), QS.model_data.Y(3,:), ...
                        'og', 'markersize', 6, 'markerfacecolor', 'g');

            hbutton=uicontrol(QS.h.hf,'style','pushbutton',...
                                  'string','Stop',...
                                  'callback','Stop()'...
                              );        
            axis([0 6 0 6 0 6]);
        else

            set(QS.h.Xpast,'XData', QS.system_data.Y(1,:), 'YData', QS.system_data.Y(2,:), 'ZData', QS.system_data.Y(3,:));
            set(QS.h.Xpred,'XData', QS.model_data.Y(1,:), 'YData', QS.model_data.Y(2,:), 'ZData', QS.model_data.Y(3,:));


        end
    end