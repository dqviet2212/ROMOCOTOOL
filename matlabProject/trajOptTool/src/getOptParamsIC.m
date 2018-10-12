function optParamsIC = getOptParamsIC(sysParams, guess)
    nStates = sysParams.nStates;
    nControls = sysParams.nControls;
    nDirColGridPts = sysParams.nDirColGridPts;
    %
    guess.states = [guess.positions; guess.velocities];    
    %
    if ((size(guess.states, 1) == nStates)&&(size(guess.states, 2) == nDirColGridPts)&&(size(guess.controls, 1) == nControls)&&(size(guess.controls, 2) == nDirColGridPts))
        optParamsIC.guess = guess;        
    else
        error('IC DIMENSION ERROR. Please verify the dimension of positions, velocities and controls!');
    end 
end