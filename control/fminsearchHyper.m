%%

% Set the optimization problem
maxITER = 200;
options = optimset('Display','iter','TolFun',1e-3,...
    'MaxIter',maxITER,'TolX',1e-3);

func = @hypermin;

gamma_i = 5000; sigma_i = 3.7;
initial = [gamma_i,sigma_i];
% initialize optimization process
[x,fval,exitflag,output] = fminsearch(func,...
      initial,options,dte,i_m,lpv_struc);

gammabary = x(1); sigmabary = x(2);

nonparmodel = PhiPhiT(dte,gammabary,sigmabary,lpv_struc,i_m,@RBFkernel);
nonparmodel.gamma = gammabary;
nonparmodel.sigma = sigmabary;

[~,nonparmodel.bfrtun,~,~] = simnonpar(dts,noparmodel,sigmabary,...
    lpv_struc,@RBFkernel);

model = nonparmodel;