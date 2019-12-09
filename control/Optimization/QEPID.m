function C = QEPID(K,sys,zeta)%ref_input;

    linear_app = sys; 
    Tsim = 100; %segundos
    
    Kc = K(1); 
    Ti = K(2);
    %Td = K(3);
    Td = 0;
    w_o = K(3);
    zeta_o = zeta;
    
    sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');
    sim('PID.slx',Tsim,sim_opts);
    
    C = (y_hat-y_ref)'*(y_hat-y_ref)/length(y_hat);
    
    % Visualizations
    figure(1)
    subplot(2,2,2)
    plot(control_signal,'k-','LineWidth',1.5); %hold on;
    %plot(control_signal_filtered); hold off;
    subplot(2,2,3)
    plot(y_hat,'r-','LineWidth',1); hold on;
    plot(y_ref,'b-','LineWidth',1.5); hold off;
    
    figure(1)
    title(['Cost function --> ',num2str(C)])
    subplot(2,2,4)
    load('PID.mat','plot_ind','x_mag')
    plot_ind = plot_ind + 1; 
    x_mag = [x_mag;Kc,Ti,Td,w_o];
    save('PID.mat')
    plot(0:1:plot_ind,x_mag(:,1),'r-','LineWidth',1.5)
    hold on
    plot(0:1:plot_ind,x_mag(:,2),'b-','LineWidth',1.5)
    plot(0:1:plot_ind,x_mag(:,3),'g-','LineWidth',1.5)
    plot(0:1:plot_ind,x_mag(:,4),'p-','LineWidth',1.5)
    legend({'Kc','Ti','Td','w_o'},'Location','northwest')
    hold off
    
end