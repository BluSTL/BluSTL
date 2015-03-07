function h = init_plot(hist_data,pred_data,Aux,nb_stages,time,legends)

    close all
        
    figure;
    subplot(4,1,1);
    hold on;
    h.X1past = plot(hist_data.time, hist_data.X(1,:), 'LineWidth',3);
    h.X2past = plot(hist_data.time, hist_data.X(3,:),'+-r', 'LineWidth',3);
    %h.X1pred = plot(pred_data.time, pred_data.T(1,:),'g--', 'LineWidth',2);
    %h.X2pred = plot(pred_data.time, pred_data.T(3,:),'m--', 'LineWidth',2);
    hl =legend('x-Position of Ego Vehicle ', 'y-Position of Adversary Vehicle');
    set(hl,'FontSize',14);
    set(gca, 'FontSize',14);
    hold on;
    grid on;

    subplot(4,1,2);
    hold on;
    h.dist1 = plot(hist_data.time,abs(hist_data.X(1,:)), 'LineWidth',3);
    h.dist2 = plot(hist_data.time,-abs(hist_data.X(3,:)),'+-r', 'LineWidth',3);
    %h.X1pred = plot(pred_data.time, pred_data.T(1,:),'g--', 'LineWidth',2);
    %h.X2pred = plot(pred_data.time, pred_data.T(3,:),'m--', 'LineWidth',2);
    legend('- Distance of Ego vehicle from intersection','+ Distance of Adversary vehicle from intersection');
    set(hl,'FontSize',14);
    set(gca, 'FontSize',14);

    hold on;
    grid on;


    subplot(4,1,3);
    hold on
    h.V1past = plot(hist_data.time, hist_data.X(2,:), 'LineWidth',3);
    h.V2past = plot(hist_data.time, hist_data.X(4,:),'+-r', 'LineWidth',3);
    %h.V1pred = plot(pred_data.time, pred_data.T(2,:),'g--', 'LineWidth',2);
    %h.V2pred = plot(pred_data.time, pred_data.T(4,:),'m--', 'LineWidth',2);
    hl = legend('Velocity of Ego Vehicle', 'Velocity of Adversary Vehicle');
    set(hl,'FontSize',14);
    set(gca, 'FontSize',14);
    grid on

    subplot(4,1,4);
    hold on

    h.Upast = stairs(hist_data.time(1:2), [hist_data.U hist_data.U], 'LineWidth',3);
    h.Wpast = stairs(hist_data.time(1:2), [hist_data.W hist_data.W],'+-r','LineWidth',3);
    %h.Upred = stairs(pred_data.time, pred_data.U, 'LineWidth',2);
    %h.Wpred = stairs(pred_data.time, pred_data.W, 'LineWidth',2);
    legend('Acceleration of Ego Vehicle','Acceleration of Adversary Vehicle');
    grid on;
    hl = xlabel('Time (s)');
    set(hl,'FontSize',14);
    set(gca, 'FontSize',14);            
end