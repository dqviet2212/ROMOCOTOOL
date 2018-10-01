function robot = getRobotEnergy(robot)
    %% Gravity acceleration
    symGravityAccel = sym('g');
    
    %% Base
    if strcmp(robot.base.type, 'fixed')
        kinematicEnergy = sym(zeros(1));
        potentialEnergy = sym(zeros(1));
    else
        baseIc = robot.base.inertia;    
        baseR = robot.base.origin.xyz;
        baseM = robot.base.mass;
        baseV = robot.base.origin.dxyz;
        baseId = baseIc + baseM*(baseR'*baseR*eye(3) - baseR*baseR');
        baseW = robot.base.origin.drpy;
        kinematicEnergy = 0.5*(baseM*(baseV'*baseV) + baseW'*baseId*baseW);
        potentialEnergy = baseM*symGravityAccel*baseR(3, 1);
    end    
    
    %% Kinematic chain    
    nKinematicChains = size(robot.kinematicChains, 2);    
    for i1 = 1:nKinematicChains        
        nJoints = size(robot.kinematicChains(i1).joint, 2);        
        for i2 = 1:nJoints
            linkIc = robot.kinematicChains(i1).link(i2).inertia;
            linkR = robot.kinematicChains(i1).link(i2).origin.xyz;
            linkM = robot.kinematicChains(i1).link(i2).mass;            
            linkV = robot.forwardKinematics.chain(i1).linkVel(:, :, i2);
            linkId = linkIc + linkM*(linkR'*linkR*eye(3) - linkR*linkR');
            linkW = robot.forwardKinematics.chain(i1).symJointVar(i2)*robot.kinematicChains(i1).joint(i2).axis;
            kinematicEnergy = kinematicEnergy + 0.5*(linkM*(linkV'*linkV) + linkW'*linkId*linkW);
            potentialEnergy = potentialEnergy + linkM*symGravityAccel*robot.forwardKinematics.chain(i1).linkPos(3, :, i2);
        end           
    end
    robot.Lagrangian.symGravityAccel = symGravityAccel;
    robot.Lagrangian.kinematicEnergy = kinematicEnergy;
    robot.Lagrangian.potentialEnergy = potentialEnergy;
end