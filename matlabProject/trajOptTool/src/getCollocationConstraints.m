function colEqCst = getCollocationConstraints(sysDynamics, optProbObj)    
    trajOpt2roModSysParams = optProbObj.trajOpt2roModSysParams;
    nDirColGridPts = optProbObj.sysParams.nDirColGridPts;
    %
    t0 = sysDynamics.t0;
    tf = sysDynamics.tf;
    states = sysDynamics.states;
    controls = sysDynamics.controls;
    deltaT = (tf - t0)/nDirColGridPts;    
    %
    colEqCst = [];
    for i1 = 1:(nDirColGridPts - 1)
        % State and control at the beginning of the time interval
        x_i = states(:, i1);
        u_i = controls(:, i1);
        % State and control at the beginning of the next time interval
        x_n = states(:, i1 + 1);
        u_n = controls(:, i1 + 1);
        % Time derivative of the state at the beginning of the time interval
        % dotx_i = getSysDynamics(x_i, u_i, sysParams);        
        [M_i, C_i, G_i] = getSysDynMat(x_i, trajOpt2roModSysParams);
        dotx_i = getStateSpaceModel(x_i, u_i, [M_i, C_i, G_i]);
        % Time derivative of the state at the beginning of the next time interval
        % dotx_n = getSysDynamics(x_n, u_n, sysParams);        
        [M_n, C_n, G_n] = getSysDynMat(x_n, trajOpt2roModSysParams);
        dotx_n = getStateSpaceModel(x_n, u_n, [M_n, C_n, G_n]);
        % State and control at the collocation nodes
        x_c = 0.5*(x_i + x_n) + 0.125*deltaT*(dotx_i - dotx_n);
        u_c = 0.5*(u_i + u_n);
        % Time derivative of the state at the collocation nodes
        % dotx_c = getSysDynamics(x_c, u_c, sysParams);       
        [M_c, C_c, G_c] = getSysDynMat(x_c, trajOpt2roModSysParams);
        dotx_c = getStateSpaceModel(x_c, u_c, [M_c, C_c, G_c]);
        % Integration defects
        colDefects = x_i - x_n + (deltaT/6)*(dotx_i + 4*dotx_c + dotx_n);        
        % Continuous state
        colEqCst = [colEqCst; colDefects]; %#ok<AGROW>
    end
end