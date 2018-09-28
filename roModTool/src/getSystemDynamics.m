function robot = getSystemDynamics(robot)
    %% Kinematic and potential energy
    Ek = robot.Lagrangian.kinematicEnergy;
    Ep = robot.Lagrangian.potentialEnergy;
    
    %% System dynamics
    states = robot.symStateVariable;
    nStates = length(states);
    nJoints = 0.5*nStates;
    positions = states(1:nJoints, 1);
    velocities = states((nJoints+1):end, 1);

    %% Dynamical matrices
    % Inertia matrix
    M = sym(zeros(nJoints, nJoints));
    for i1 = 1:nJoints
        for j1 = 1:nJoints
            % M(i1, j1) = simplify(diff(diff(Ek, velocities(j1, 1)), velocities(i1, 1)));
            M(i1, j1) = diff(diff(Ek, velocities(j1, 1)), velocities(i1, 1));
        end
    end
    % Coriolis matrix
    C1 = sym(zeros(nJoints, 1));
    C2 = sym(zeros(nJoints, 1));
    C3 = sym(zeros(nJoints, 1));
    C = sym(zeros(nJoints, nJoints));
    for i1 = 1:nJoints
        for j1 = 1:nJoints        
            C4 = 0;
            for k = 1:nJoints            
                C1(k, 1) = 0.5*diff(M(i1, j1), positions(k, 1))*velocities(k, 1);
                C2(k, 1) = 0.5*diff(M(i1, k), positions(j1, 1))*velocities(k, 1);
                C3(k, 1) = 0.5*diff(M(j1, k), positions(i1, 1))*velocities(k, 1);            
                C4 = C4 + C1(k) + C2(k) - C3(k);
            end
            % C(i1, j1) = simplify(C4);
            C(i1, j1) = C4;
        end
    end
    % Gravity matrix
    G = sym(zeros(nJoints, 1));
    for i1 = 1:nJoints
        % G(i1, 1) = simplify(diff(Ep, positions(i1)));
        G(i1, 1) = diff(Ep, positions(i1));
    end    

    %% Verification
    checkM = sym(zeros(nJoints, nJoints));
    % Symmetric property of the initia matrix, i.e. M(i, j) - M(j, i) = 0
    symM = sym(zeros(nJoints, nJoints));
    for i1 = 1:nJoints
        for j1 = 1:nJoints
            % symM(i1, j1) = simplify(M(i1, j1) - M(j1, i1));
            symM(i1, j1) = M(i1, j1) - M(j1, i1);
        end
    end    
    % Anti-symmetric propety of the matrix aM = diff(M, t) - 2C, i.e. aM(i, j) + aM(j, i) = 0
    positionsT = sym(zeros(nJoints, 1));
    for i1 = 1:nJoints
        positionsT(i1, 1) = str2sym([char(positions(i1, 1)), '(t)']);
    end
    t = sym('t');
    velocitiesT = diff(positionsT, t);
    aM = sym(zeros(nJoints, nJoints));
    for i1 = 1:nJoints
        for j1 = 1:nJoints
            aM(i1, j1) = subs(subs(diff(subs(M(i1, j1), positions, positionsT), t), velocitiesT, velocities), positionsT, positions) - 2*C(i1,j1);
        end
    end
    asymM = sym(zeros(nJoints, nJoints));
%     for i1 = 1:nJoints
%         for j1 = 1:nJoints
%             % asymM(i1, j1) = simplify(aM(i1, j1) + aM(j1, i1));
%             asymM(i1, j1) = aM(i1, j1) + aM(j1, i1);
%         end
%     end
    passCond1 = isequaln(checkM, symM);
    passCond2 = isequaln(checkM, asymM);
    if (passCond1&&passCond2)                
%         activeConInd = find(obj.controls~=0);
%         if isempty(activeConInd)
%             activeConInd = 0;
%         end
%         passiveConInd = find(obj.controls==0);
%         if isempty(passiveConInd)
%             passiveConInd = 0;
%         end
%         nControls = length(activeConInd);
        %
        robot.systemDynamics.nStates = nStates;
%         robot.systemDynamics.nControls = nControls;
%         robot.systemDynamics.activeConInd = activeConInd;
%         robot.systemDynamics.passiveConInd = passiveConInd;
        robot.systemDynamics.M = M;
        robot.systemDynamics.C = C;
        robot.systemDynamics.G = G;
    else
        if ~passCond1
            error('Inertia matrix is not symmetric');
        end
        if ~passCond2
            error('Anti-symmetric property is not satisfied');    
        end
    end
end