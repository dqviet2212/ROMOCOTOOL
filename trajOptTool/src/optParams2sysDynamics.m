function sysDynamics = optParams2sysDynamics(optParams, optProbObj) 
    nStates = optProbObj.sysParams.nStates;
    nControls = optProbObj.sysParams.nControls;
    nDirColGridPts = optProbObj.sysParams.nDirColGridPts;
    %
    t0 = optParams(1);
    tf = optParams(2);
    states = reshape(optParams(3:(nStates*nDirColGridPts+2)), [nDirColGridPts, nStates])';
    controls = reshape(optParams((nStates*nDirColGridPts+3):end), [nDirColGridPts, nControls])';    
    sysDynamics.t0 = t0;
    sysDynamics.tf = tf;
    sysDynamics.states = states;
    sysDynamics.controls = controls;
end