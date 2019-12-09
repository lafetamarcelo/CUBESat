function C = QELandL(K,sys)%ref_input);
    linear_app = sys; 
    Tsim = 100; %segundos
    
    pr1 = K(1); pi1 = K(2)*1i;
    zr = K(3); ks = K(4);
    
    sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');
    sim('LandL.slx',Tsim,sim_opts);
    
    C = (y_hat-y_ref)'*(y_hat-y_ref)/length(y_hat);
    
    % Visualizations
    figure(1)
    subplot(2,2,2)
    plot(control_signal,'k-','LineWidth',1.5); %hold on;
    %plot(control_signal_filtered); hold off;
    subplot(2,2,3)
    plot(y_hat,'r--','LineWidth',1.5); hold on;
    plot(y_ref,'b-','LineWidth',1.5); hold off;
    
    figure(1)
    title(['Cost function --> ',num2str(C)])
    subplot(2,2,4)
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    
    sz = (100 - C)/2;
    if (sz > 50)
        sz = 50;
    elseif (sz < 0)
        sz = 2;
    end
    
    if K(1) < 100 && K(2) < 100
       scatter(-K(1),K(2),sz,'r','x')
       hold on
       scatter(-K(1),-K(2),sz,'r','x')
       hold on
    end
    
    scatter(ks,0,sz,'g','*')
    hold on 
    scatter(-zr,0,sz,'k','o')
    
end