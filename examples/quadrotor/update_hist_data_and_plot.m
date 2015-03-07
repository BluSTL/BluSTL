function [hist_data, pred_data] = update_hist_data_and_plot(h,hist_data,time_new,u_new,x_new,w_new,pred_data,time_d,Xpred,Upred,Auxn)

    dim = 3;
    
    if 3 == dim
        C = [1 zeros(1, 9); 
             0 1 zeros(1, 8);
             0 0 1 zeros(1, 7)];
    elseif 2 == dim
    	C = [1 zeros(1, 9); 
             0 1 zeros(1, 8)];
    end
    
    hist_data.time(end+1) = time_new/60;
    hist_data.U(:,end+1) = u_new;
    hist_data.X(:,end+1) = x_new;
    hist_data.Xf(:,end+1) = C*hist_data.X(:,end);

    pred_data.time = time_d/60;
    pred_data.Xf = C*double(Xpred);
    pred_data.U = double(Upred);
    

    set(h.Xpred,'XData', pred_data.Xf(1,:), 'YData', pred_data.Xf(2,:), 'ZData', pred_data.Xf(3,:));
   
    set(h.Xpast,'XData', hist_data.Xf(1,:), 'YData', hist_data.Xf(2,:), 'ZData', hist_data.Xf(3,:));
   
end