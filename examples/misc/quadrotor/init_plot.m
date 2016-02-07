function h = init_plot(hist_data,pred_data,Aux,nb_stages,time,legends)

    h = struct();
    %% Create the environment
    ex = 5;
    env = envLTL(ex);
    obstacles = env.work.unsafe;
    h.hf = plotEnv(env, obstacles);

    hold on;
    zlabel('z (m)');
    h.Xpast = plot3(hist_data.Xf(1,:)', hist_data.Xf(2,:)', hist_data.Xf(3,:)', ...
                'ob', 'markersize', 6, 'markerfacecolor', 'b');
    hold on;
    h.Xpred = plot3(pred_data.Xf(1,:),pred_data.Xf(2,:), pred_data.Xf(3,:), ...
                'og', 'markersize', 6, 'markerfacecolor', 'g');
  
    hbutton=uicontrol(h.hf,'style','pushbutton',...
                          'string','Stop',...
                          'callback','Stop()'...
                      );        
                  
   
end