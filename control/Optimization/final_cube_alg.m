%% Load and pre-treat data

clear; close all; clc;

%% READING AND PROCESSING THE EXPERIMENTAL DATA

%% Read and handly remove data

% Load the experimental data -- CUBESat measures
load('secondary_test.mat')

% Collect the test and validation data
Out = [Out_t; Out_v];        % cube input
In = [In_t; In_v];           % cube output
Set_Point = [Set_t; Set_v];  % set--point

time = (0:1:length(In)-1)*Ts;

% Remove the noise data, and put the cube on the axis origin i.e. removes
% the DC level of each signal, without the first ones that are actually
% noise, due to the turn on of the cube.
Out_t = Out_t(5:end) - Out_t(5);
In_t = In_t(5:end) - In_t(5);

Out_v = Out_v(5:end) - Out_v(15);
In_v = In_v(5:end) - In_v(15);


% Plot the signals to get a notion, about what is happening on the
% experiment.
figure(1)
subplot(2,2,1)
plot(time,In,'r','LineWidth',1.5)
hold on
plot(time,Set_Point,'b--')
hold off
subplot(2,2,2)
plot(time,Out,'r','LineWidth',1.5)
subplot(2,2,3)
plot(time(1:length(In_t)),In_t,'r','LineWidth',1.5)
subplot(2,2,4)
plot(time(1:length(Out_t)),Out_t,'r','LineWidth',1.5)


%% SPA data analysis
% Just do a simple frequency analysis on the data, to understand noise
% patterns and if the identification process further will actually be
% correct and will learn most colored stuff.

data_t = iddata(Out_t,In_t,Ts);
g_t = spa(data_t);
data_v = iddata(Out_v,In_v,Ts);
g_v = spa(data_v);

figure(1)
subplot(2,2,3)
bode(g_t)
subplot(2,2,4)
bode(g_v)

%% IDENTIFICATION SECTION
% In this section several models are used to learn the behavior of the
% CUBESAT moviment. So lets use several approaches:

%% Estimate using BJ model

data_t = iddata(Out_t,In_t,Ts);
data_v = iddata(Out_v,In_v,Ts);
nb = 3; nc = 3; nd = 2; nf = 2; nk = 1;
order = [nb, nc, nd, nf, nk];
sys_bj = bj(data_t, order, 'Focus','simulation');

sys_bj.report.fit

sys_sec_bj = bj(data_t,sys_bj);

figure(2)
subplot(3,2,1)
compare(data_t,sys_bj,'r',sys_sec_bj,'g--')
subplot(3,2,2)
compare(data_v,sys_bj,'r',sys_sec_bj,'g--')

%% Estimate using ARX model

data_t = iddata(Out_t,In_t,Ts);

na = 2; nb = 3; nk = 1;
in_delay = 1; io_delay = 1;

opt = arxOptions;
opt.InitialCondition = 'estimate';
opt.Display = 'off';
opt.EnforceStability = true;
opt.Focus = 'simulation';

sys_arx = arx(data_t,[na,nb,nk],opt,...
    'InputDelay',in_delay,...
    'IODelay',io_delay);

sys_sec_arx = arx(data_t,sys_arx);
    
figure(2)
subplot(3,2,3)
compare(data_t,sys_arx,'r',sys_sec_arx,'g--')
subplot(3,2,4)
compare(data_v,sys_arx,'r',sys_sec_arx,'g--')

%% Estimate using ARMAX model

data_t = iddata(Out_t,In_t,Ts);

na = 2; nb = 2; nk = 1; nc = 1;
in_delay = 2; io_delay = 1;

opt = arxOptions;
opt.InitialCondition = 'estimate';
opt.Display = 'off';
opt.EnforceStability = true;
opt.Focus = 'simulation';

sys_arm = armax(data_t,[na,nb,nc,nk],opt,...
    'InputDelay',in_delay,...
    'IODelay',io_delay,'IntegrateNoise',1);

sys_sec_arm = armax(data_t,sys_arm);
    
figure(2)
subplot(3,2,5)
compare(data_t,sys_arm,'r',sys_sec_arm,'g--')
subplot(3,2,6)
compare(data_v,sys_arm,'r',sys_sec_arm,'g--')

%% Estimate using nonlinear Hammerstein-Wiener model

data_v = iddata(Out_t,In_t,Ts);
data_t = iddata(Out_v,In_v,Ts);
opt = oeOptions('Focus','simulation',...
    'InitialCondition','zero',...
    'EnforceStability',true);

nb = 3; nf = 3; nk = 2;

LinearModel = oe(data_t,[nb nf nk],opt);

opt_nl = nlhwOptions;
opt_nl.Regularization.Lambda = 0.1;
%opt_nl.Regularization.R = 0.01*eye(25);
opt_nl.SearchMethod = 'lsqnonlin';

OutputNL = wavenet;
OutputNL.NumberOfUnits = 2;

InputNL = sigmoidnet;
InputNL.NumberOfUnits = 85;

sys = nlhw(data_t,LinearModel,InputNL,OutputNL,opt_nl);

sec_sys = nlhw(data_t,sys);

figure(3)
subplot(2,2,1)
compare(data_t,sys,'r',sec_sys,'g--')
subplot(2,2,2)
compare(data_v,sys,'r',sec_sys,'g--')
subplot(2,2,3)
step(sys)
subplot(2,2,4)
step(sec_sys)


%% AUTOMATIC CONTROLLER TUNNING USING COSIMULATION SECTION 
% Here is presented the section where several algorithms are used to try to
% determine the best controller parameters for the position loop of the
% CUBESAT. An approach using the linear models estimated from the above
% section as the real CUBESAT and one using the non--linear models are 
% presented in this section. One may select what controller wants to use
% for example, here is used or a Lead and Lag (@QELandL) or using a PID
% controller (@QEPID).


%% Co--simulation linear approach
close all; clear; clc;

% select the controller structure (@QEPID or @QELandL)
func = @QELandL;

% Initialize the controller parameters
if strcmp(func2str(func), 'QEPID') 
    plot_ind = 0; x_mag = zeros(1,4);
    save('PID.mat');
    clear plot_ind; clear x_mag;
    func = @QEPID;
    k_co = 8; t_io = 2; %t_do = 2;
    w_o = 20; zeta_o = 0.707;
    %initial = [k_co, t_io, t_do, w_o];
    initial = [k_co, t_io, w_o];
elseif strcmp(func2str(func), 'QELandL')    
    p_r1 = 6.94; p_i1 = 4.497;
    z_r = 1.50; kc = -21.34;
    initial = [p_r1, p_i1, z_r, kc];
end

% open data for co-simulation
load('estimate_3.mat')

figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,2,1)
compare(data_t,linear_app,'r--')

% Create reference input
%ref_input = timeseries(Out_t,...
%    0:Ts:(length(Out_t)-1)*Ts);

% Set the optimization problem
maxITER = 200;
options = optimset('Display','iter','TolFun',1e-3,...
    'MaxIter',maxITER,'TolX',1e-3);

% initialize optimization process
if strcmp(func2str(func),'QEPID')
    [x,fval,exitflag,output] = fminsearch(func,...
        initial,options,linear_app,zeta_o);%ref_input);
elseif strcmp(func2str(func),'QELandL')
    [x,fval,exitflag,output] = fminsearch(func,...
        initial,options,linear_app);
end

% simulate the best result
sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');

if strcmp(func2str(func),'QEPID') 
    Kc = x(1); 
    Ti = x(2); 
    Td = x(3);
    w_o = x(4);
    sim('PID.slx',100,sim_opts);
elseif strcmp(func2str(func),'QELandL')    
    pr1 = x(1); pi1 = x(2)*1i; 
    zr = x(3); ks = x(4);
    sim('LandL.slx',100,sim_opts);
end

if strcmp(func2str(func),'QELandL')
figure(3)
subplot(2,1,1)
title('Reference and response signals')
plot(y_ref,'b-','LineWidth',1.5)
hold on
plot(y_hat,'r--','LineWidth',1.5)
hold off
subplot(2,1,2)
title('Controller poles, zero and gain.')
ax = gca;
scatter(-x(1),x(2),80,'r','x','LineWidth',2)
hold on
scatter(-x(1),-x(2),80,'r','x','LineWidth',2)
hold on
scatter(x(4),0,80,'g','*','LineWidth',2)
hold on 
scatter(-x(3),0,80,'k','o','LineWidth',2)
ax.XAxisLocation = 'origin';
ax.XScale = 'log';
ax.YAxisLocation = 'origin';
legend({'pole','pole','gain','zero'},'Location',...
    'northwest','NumColumns',2);
end

%% Non--linear optimization
close all; clear; clc;

% select the controller structure (@QEPID or @QELandL)
func = @QELandL;

% Initialize the controller parameters
if strcmp(func2str(func), 'QEPID') 
    plot_ind = 0; x_mag = zeros(1,4);
    save('PID.mat');
    clear plot_ind; clear x_mag;
    func = @QEPID;
    k_co = 8; t_io = 2; t_do = 2;
    w_o = 20; zeta_o = 0.9;
    initial = [k_co, t_io, t_do, w_o];
elseif strcmp(func2str(func), 'QELandL')    
    p_r1 = 6.94; p_i1 = 4.497;
    z_r = 1.50; kc = -21.34;
    initial = [p_r1, p_i1, z_r, kc];
end

% open data for co-simulation
load('estimate_3.mat')

figure('units','normalized','outerposition',[0 0 1 1])
subplot(2,2,1)
compare(data_t,sec_sys,'r--')

% Create reference input
%ref_input = timeseries(Out_t,...
%    0:Ts:(length(Out_t)-1)*Ts);

% Set the optimization problem
maxITER = 200;
options = optimset('Display','iter','TolFun',1e-3,...
    'MaxIter',maxITER,'TolX',1e-3);

% initialize optimization process
if strcmp(func2str(func),'QEPID')
    [x,fval,exitflag,output] = fminsearch(func,...
        initial,options,sec_sys,zeta_o);%ref_input);
elseif strcmp(func2str(func),'QELandL')
    [x,fval,exitflag,output] = fminsearch(func,...
        initial,options,sec_sys);
end
% simulate the best result
sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');

if strcmp(func2str(func),'QEPID') 
    Kc = x(1); 
    Ti = x(2); 
    Td = x(3);
    w_o = x(4);
    linear_app = sec_sys;
    sim('PID.slx',100,sim_opts);
elseif strcmp(func2str(func),'QELandL')    
    pr1 = x(1); pi1 = x(2)*1i; 
    zr = x(3); ks = x(4);
    linear_app = sec_sys;
    sim('LandL.slx',100,sim_opts);
end

if strcmp(func2str(func),'QELandL')
figure(3)
subplot(2,1,1)
title('Reference and response signals')
plot(y_ref,'b-','LineWidth',1.5)
hold on
plot(y_hat,'r--','LineWidth',1.5)
hold off
subplot(2,1,2)
title('Controller poles, zero and gain.')
ax = gca;
scatter(-x(1),x(2),80,'r','x','LineWidth',2)
hold on
scatter(-x(1),-x(2),80,'r','x','LineWidth',2)
hold on
scatter(x(4),0,80,'g','*','LineWidth',2)
hold on 
scatter(-x(3),0,80,'k','o','LineWidth',2)
ax.XAxisLocation = 'origin';
ax.XScale = 'log';
ax.YAxisLocation = 'origin';
legend({'pole','pole','gain','zero'},'Location',...
    'northwest','NumColumns',2);
end


% %% -----------------------------------------
% function C = QELandL(K,sys)%ref_input);
% 
%     linear_app = sys; 
%     Tsim = 100; %segundos
%     
%     pr1 = K(1); pi1 = K(2)*1i;
%     zr = K(3); ks = K(4);
%     
%     sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');
%     sim('LandL.slx',Tsim,sim_opts);
%     
%     C = (y_hat-y_ref)'*(y_hat-y_ref)/length(y_hat);
%     
%     % Visualizations
%     figure(1)
%     subplot(2,2,2)
%     plot(control_signal,'k-','LineWidth',1.5); %hold on;
%     %plot(control_signal_filtered); hold off;
%     subplot(2,2,3)
%     plot(y_hat,'r--','LineWidth',1.5); hold on;
%     plot(y_ref,'b-','LineWidth',1.5); hold off;
%     
%     figure(1)
%     title(['Cost function --> ',num2str(C)])
%     subplot(2,2,4)
%     ax = gca;
%     ax.XAxisLocation = 'origin';
%     ax.YAxisLocation = 'origin';
%     
%     sz = (100 - C)/2;
%     if (sz > 50)
%         sz = 50;
%     elseif (sz < 0)
%         sz = 2;
%     end
%     
%     if K(1) < 100 && K(2) < 100
%        scatter(-K(1),K(2),sz,'r','x')
%        hold on
%        scatter(-K(1),-K(2),sz,'r','x')
%        hold on
%     end
%     
%     scatter(ks,0,sz,'g','*')
%     hold on 
%     scatter(-zr,0,sz,'k','o')
%     
% end
% 
% %% 
% function C = QEPID(K,sys)%ref_input;
% 
%     linear_app = sys; 
%     Tsim = 100; %segundos
%     
%     k_c = K(1); 
%     t_i = K(2);
%     t_d = K(3);
%     
%     sim_opts = simset('DstWorkspace','current','SrcWorkspace','current');
%     sim('PID.slx',Tsim,sim_opts);
%     
%     C = (y_hat-y_ref)'*(y_hat-y_ref)/length(y_hat);
%     
%     % Visualizations
%     figure(1)
%     subplot(2,2,2)
%     plot(control_signal,'k-','LineWidth',1.5); %hold on;
%     %plot(control_signal_filtered); hold off;
%     subplot(2,2,3)
%     plot(y_hat,'r-','LineWidth',1); hold on;
%     plot(y_ref,'b-','LineWidth',1.5); hold off;
%     
%     figure(1)
%     title(['Cost function --> ',num2str(C)])
%     subplot(2,2,4)
%     load('PID.mat','plot_ind','x_mag')
%     plot_ind = plot_ind + 1; 
%     x_mag = [x_mag;k_c,t_i,t_d];
%     save('PID.mat')
%     plot(1:1:lenght(plot_ind),x_mag(:,1),'r-','LineWidth',1.5)
%     hold on
%     plot(1:1:lenght(plot_ind),x_mag(:,2),'b-','LineWidth',1.5)
%     plot(1:1:lenght(plot_ind),x_mag(:,3),'g-','LineWidth',1.5)
%     legend({'Kc','Ti','Td'},'Location','northwest')
%     hold off
%     
% end


