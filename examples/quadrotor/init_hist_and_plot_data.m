function [hist_data, pred_data] = init_hist_and_plot_data(hist_data,pred_data,time_new,x_new,w_new,Xpred,Upred,time_d,x0,Auxn)
% first step, we just computed the inputs for the initial horizon

    dim = 3;
    if 3 == dim
        C = [1 zeros(1, 9); 
             0 1 zeros(1, 8);
             0 0 1 zeros(1, 7)];
    elseif 2 == dim
        C = [1 zeros(1, 9); 
             0 1 zeros(1, 8)];
    end
    
    hist_data.time = [0 time_new]/60;
    hist_data.X = [x0 x_new]; 
    hist_data.Xf = C*hist_data.X;
    hist_data.U = double(Upred(:,1));                
    
    pred_data.time = time_d/60;
    pred_data.Xf = C*double(Xpred);
    pred_data.U = double(Upred);
    
end