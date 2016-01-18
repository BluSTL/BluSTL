function Sys = STLC_update_plot(Sys)
% STLC_update_plot updates the plots of an STLC_lti at runtime
%
% Input: Sys, the STLC_lti instance
% Output: Sys with plots modified
%
% :copyright: TBD
% :license: TBD

system_style= {'LineWidth',2};
model_style = {'--', 'Color', [0. 0.5 0.],'LineWidth', 2};
axis_style = {'Fontsize',12};

try % try updating existing plots
    for iX = Sys.plot_x
        set(Sys.h.Xpast(iX), 'Xdata', Sys.system_data.time, 'Ydata',Sys.system_data.X(iX,:));
        set(Sys.h.Xmodel(iX), 'Xdata', Sys.model_data.time,'Ydata', Sys.model_data.X(iX,:));
    end
    
    for iY = Sys.plot_y
        set(Sys.h.Ypast(iY), 'Xdata', Sys.system_data.time,'Ydata',  Sys.system_data.Y(iY,:));
        set(Sys.h.Ymodel(iY), 'Xdata', Sys.model_data.time(1:end-1),'Ydata', Sys.model_data.Y(iY,:));
    end
    
    for iU = Sys.plot_u
        set(Sys.h.Upast(iU), 'Xdata', Sys.system_data.time,'Ydata',  Sys.system_data.U(iU,:));
        set(Sys.h.Umodel(iU), 'Xdata', Sys.model_data.time(1:end-1),'Ydata', Sys.model_data.U(iU,:));
    end
    
    for iW = Sys.plot_w
        set(Sys.h.Wpast(iW), 'Xdata', Sys.system_data.time,'Ydata',  Sys.system_data.W(iW,:));
        set(Sys.h.Wmodel(iW), 'Xdata', Sys.model_data.time(1:end-1),'Ydata', Sys.model_data.W(iW,1:end-1));
    end
    xlabel('Time');
    
catch % updading didn't work (either first call, or figure was closed or some other error), then create new plot
    Sys.h =[];
    Sys.h.hf = figure;
    nb_plots = numel(Sys.plot_x)+numel(Sys.plot_y)+numel(Sys.plot_u)+numel(Sys.plot_w);
    cur_plot = 1;
    
    for iX = Sys.plot_x
        subplot(nb_plots,1,cur_plot);
        hold on; grid on;
        if (~isempty(Sys.system_data.time))
            Sys.h.Xpast(iX) = plot(Sys.system_data.time, Sys.system_data.X(iX,1:end),system_style{:});
        end
        if (~isempty(Sys.model_data.time))
            Sys.h.Xmodel(iX) = stairs(Sys.model_data.time,Sys.model_data.X(iX,:), model_style{:});
        end
        ylabel(Sys.xlabel{iX});
        cur_plot = cur_plot+1;
        set(gca,axis_style{:});
    end
    
    for iY = Sys.plot_y
        subplot(nb_plots,1,cur_plot);
        hold on;grid on;
        if (~isempty(Sys.system_data.time))
            
            Sys.h.Ypast(iY) = plot(Sys.system_data.time, Sys.system_data.Y(iY,:),system_style{:});
        end
        if (~isempty(Sys.model_data.time))
            Sys.h.Ymodel(iY) = stairs(Sys.model_data.time(1:end-1), Sys.model_data.Y(iY,:), model_style{:});
        end
        ylabel(Sys.ylabel{iY});
        cur_plot = cur_plot+1;
        set(gca,axis_style{:});
    end
    
    for iU = Sys.plot_u
        subplot(nb_plots,1,cur_plot);
        hold on;grid on;
        if (~isempty(Sys.system_data.time))
            Sys.h.Upast(iU) = stairs(Sys.system_data.time, Sys.system_data.U(iU,:),system_style{:});
        end
        if (~isempty(Sys.model_data.time))
            Sys.h.Umodel(iU) = stairs(Sys.model_data.time(1:end-1),Sys.model_data.U(iU,:), model_style{:});
        end
        ylabel([Sys.ulabel{iU}]);
        cur_plot = cur_plot+1;
        set(gca,axis_style{:});
    end
    
    for iW = Sys.plot_w
        subplot(nb_plots,1,cur_plot);
        hold on;grid on;
        if (~isempty(Sys.system_data.time))
            
            Sys.h.Wpast(iW) = plot(Sys.system_data.time, Sys.system_data.W(iW,:),system_style{:});
        end
        if (~isempty(Sys.model_data.time))
            Sys.h.Wmodel(iW) = stairs(Sys.model_data.time(1:end-1),Sys.model_data.W(iW,1:end-1), model_style{:});
        end
        ylabel(Sys.wlabel{iW});
        cur_plot = cur_plot+1;
        set(gca,axis_style{:});
    end
    
    if Sys.stop_button
        Sys.h.hbutton=uicontrol(Sys.h.hf,'style','pushbutton',...
            'string','Stop',...
            'callback','Stop()'...
            );
    end
end
end
