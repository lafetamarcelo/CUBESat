function bfr = hypermin(K,dte,i_m,lpv_struc)
    
    dts.y = dte.y{i_m};
    dts.u = dte.u{i_m};
    dts.p = dte.p{i_m};
    dts.n = length(dte.y{i_m});
    dts.initial = dte.initial{i_m};
    dts.final = dte.final{i_m};
    
    gamma = K(1); sigma = K(2);
    
    model = PhiPhiT(dte,gamma,sigma,lpv_struc,i_m,@RBFkernel);  
    % determine the performance of the estimated model
    [~,bfr,~,~] = simnonpar(dts,model,sigma,lpv_struc,@RBFkernel);
    
    figure(1)
    scatter(gamma,sigma,[],'MarkerFaceColor',[bfr/100 0 0],'filled')
    hold on
end