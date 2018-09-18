function stepEqCst = getStepEqualityConstraints(sysDynamics, optProbObj)
    states = sysDynamics.states;    
    stepEqCst = [states(:, 1) - optProbObj.desIniState;
                 states(:, end) - optProbObj.desFinState;
                 ];

end
            