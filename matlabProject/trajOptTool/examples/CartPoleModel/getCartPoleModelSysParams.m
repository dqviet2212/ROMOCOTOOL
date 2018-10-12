function sysParams = getCartPoleModelSysParams()
    nStates = 4;            % Number of state variable
    nControls = 1;          % Number of controls
    nDirColGridPts = 15;    % Number of direct collocation grid points
    m1 = 1;     % Mass of the cart [kg]
    m2 = 0.3;   % Mass of the pole [kg]
    l = 0.5;    % Length of the pole [m]
    g = 9.81;   % Gravity [m.s-2]
    dmax = 2.0; % Max displacement of the cart
    d = 1.0;    % Desired displacement of the cart
    umax = 20;  % Max control effort    
    %
    sysParams.nStates = nStates;
    sysParams.nControls = nControls;
    sysParams.nDirColGridPts = nDirColGridPts;
    sysParams.m1 = m1;
    sysParams.m2 = m2;
    sysParams.l = l;
    sysParams.g = g;
    sysParams.dmax = dmax;
    sysParams.d = d;
    sysParams.umax = umax;
end