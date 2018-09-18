function cost = getOptObjective(sysDynamics, optProbObj)
    t0 = sysDynamics.t0;
    tf = sysDynamics.tf;
    controls = sysDynamics.controls;
    cost = sum(controls.^2) + (tf - t0).^2;
end