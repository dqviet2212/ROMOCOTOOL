function optParams = sysDynamics2optParams(sysDynamics, optProbObj)     
    nStates = optProbObj.sysParams.nStates;
    nControls = optProbObj.sysParams.nControls;
    nDirColGridPts = optProbObj.sysParams.nDirColGridPts;
    %
    t0 = sysDynamics.t0;
    tf = sysDynamics.tf;
    states = sysDynamics.states;
    controls = sysDynamics.controls;    
    optParams = [t0; tf; reshape(states', [nStates*nDirColGridPts, 1]); reshape(controls', [nControls*nDirColGridPts, 1])];
end