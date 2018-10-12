function interpSysDynamics = getSysDynamicsInterpolation(optSysDynamics, optProbObj)
    t0 = optSysDynamics.t0;
    tf = optSysDynamics.tf;
    states = optSysDynamics.states;
    controls = optSysDynamics.controls;
    nStates = optProbObj.sysParams.nStates;
    nControls = optProbObj.sysParams.nControls;
    nDirColGridPts = optProbObj.sysParams.nDirColGridPts;
    %
    t = linspace(t0, tf, nDirColGridPts);
    positions = states(1:(0.5*nStates), :);
    velocities = states((0.5*nStates + 1):end, :);

    %% Interpolation
    Ts = 1e-3;
    time = t0:Ts:tf;
    if (nStates >= 2)
        interpPositions = interp1(t, positions', time, 'spline')';
        interpVelocities = interp1(t, velocities', time, 'spline')';
    else
        interpPositions = interp1(t, positions, time, 'spline');
        interpVelocities = interp1(t, velocities, time, 'spline');
    end
    if (nControls >= 2)
        interpControls = interp1(t, controls', time, 'spline')';
    else
        interpControls = interp1(t, controls, time, 'spline');
    end    
    %
    interpSysDynamics.time = time;
    interpSysDynamics.positions = interpPositions;
    interpSysDynamics.velocities = interpVelocities;
    interpSysDynamics.controls = interpControls;
end