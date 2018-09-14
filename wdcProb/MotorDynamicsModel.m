function Output = MotorDynamicsModel(Inputs)
    %% Physical parameters
    etaM = 0.69;    % Motor efficiency
    etaG = 0.9;     % Gearbox efficiency
    Kt = 7.67e-3;   % Torque constant [NmA-1]
    Kg = 70;        % Gearbox ratio
    Kv = 7.67e-3;   % Back EMF constant [Vsrad-1]
    Jm = 3.87e-7;   % Moment of inertia of the motor rotor [kgm2]
    R = 2.6;        % Motor armature resistance [Ohm]
    %
    coefU = (etaM*etaG*Kt*Kg)/(R);
    coefDtheta = (-etaM*etaG*Kt*Kv*Kg^2)/R;
    coefD2theta = etaG*Jm*Kg^2;

    %% Dynamics equation
    dtheta = Inputs(1);
    u = Inputs(2);
    Tload = Inputs(3);
    d2theta = (coefU*u + coefDtheta*dtheta - Tload)/coefD2theta;
    Output = d2theta;
end