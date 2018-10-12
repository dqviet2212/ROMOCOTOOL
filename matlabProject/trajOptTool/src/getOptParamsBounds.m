function optParamsBounds = getOptParamsBounds(sysParams, lb, ub)
    nStates = sysParams.nStates;
    nControls = sysParams.nControls;
    nDirColGridPts = sysParams.nDirColGridPts;
    %
    lb.states = [lb.positions; lb.velocities];
    ub.states = [ub.positions; ub.velocities];
    %
    if ((size(lb.states, 1) == nStates)&&(size(lb.states, 2) == nDirColGridPts)&&(size(ub.controls, 1) == nControls)&&(size(ub.controls, 2) == nDirColGridPts))
        optParamsBounds.lb = lb;
        optParamsBounds.ub = ub;
    else
        error('BOUNDS DIMENSION ERROR. Please verify the dimension of positions, velocities and controls!');
    end 
end