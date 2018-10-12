function robot = getForwardKinematics(robot)
    %% Base
    basePos = robot.base.origin.xyz;
    baseOri = robot.base.origin.rpy;    
    dBasePos = robot.base.origin.dxyz;
    dBaseOri = robot.base.origin.drpy;
    baseRot = Rpy2Mat(baseOri);
    
    %% Kinematic chain
    nKinematicChains = size(robot.kinematicChains, 2);
    nChainJoints = zeros(nKinematicChains, 1);
    for i1 = 1:nKinematicChains
        tmpNChainJoints = 6 + size(robot.kinematicChains(i1).joint, 2);
        if strcmp(robot.base.type, 'fixed')
            tmpNChainJoints = tmpNChainJoints - 6;            
        end
        nChainJoints(i1) = tmpNChainJoints;
    end
    nChainJoints = [0; nChainJoints];
    for i1 = nKinematicChains:-1:1
        nJoints = size(robot.kinematicChains(i1).joint, 2);
        % Symbolic variable definition
        jointRot = sym(zeros(3, 3, nJoints));
        jointPos = sym(zeros(3, 1, nJoints));
        jointVel = sym(zeros(3, 1, nJoints));
        linkRot = sym(zeros(3, 3, nJoints));
        linkPos = sym(zeros(3, 1, nJoints));
        linkVel = sym(zeros(3, 1, nJoints));
        jointVar = sym(['q', num2str(i1)], [nJoints, 1]);
        dJointVar = sym(['dq', num2str(i1)], [nJoints, 1]);        
        % Extended joint variable
        if strcmp(robot.base.type, 'fixed')
            exJointVar = jointVar;
            dExJointVar = dJointVar;
            jointPosJaco = sym(zeros(3, nJoints, nJoints));
            linkPosJaco = sym(zeros(3, nJoints, nJoints));
        else
            exJointVar = [basePos; baseOri; jointVar];
            dExJointVar = [dBasePos; dBaseOri; dJointVar];
            jointPosJaco = sym(zeros(3, 6 + nJoints, nJoints));
            linkPosJaco = sym(zeros(3, 6 + nJoints, nJoints));
        end
        % Joint1
        jointRot(:, :, 1) = baseRot*Rpy2Mat(robot.kinematicChains(i1).joint(1).origin.rpy);
        jointPos(:, :, 1) = (basePos + baseRot*robot.kinematicChains(i1).joint(1).origin.xyz);
        jointPosJaco(:, :, 1) = jacobian(jointPos(:, :, 1), exJointVar);
        jointVel(:, :, 1) = jointPosJaco(:, :, 1)*dExJointVar;
        % Link1
        link1Rq = strcmp(robot.kinematicChains(i1).joint(1).type, 'revolute')*Rpy2Mat(robot.kinematicChains(i1).joint(1).axis*jointVar(1)) + strcmp(robot.kinematicChains(i1).joint(1).type, 'prismatic')*eye(3);
        linkRot(:, :, 1) = jointRot(:, :, 1)*Rpy2Mat(robot.kinematicChains(i1).link(1).origin.rpy)*link1Rq;
        linkPos(:, :, 1) = jointPos(:, :, 1) + jointRot(:, :, 1)*link1Rq*robot.kinematicChains(i1).link(1).origin.xyz;
        linkPosJaco(:, :, 1) = jacobian(linkPos(:, :, 1), exJointVar);
        linkVel(:, :, 1) = linkPosJaco(:, :, 1)*dExJointVar;        
        % Other Joints and Links
        for i2 = 2:nJoints
            % Joints
            jointRrpy = Rpy2Mat(robot.kinematicChains(i1).joint(i2-1).origin.rpy);
            jointRq = strcmp(robot.kinematicChains(i1).joint(i2-1).type, 'revolute')*Rpy2Mat(robot.kinematicChains(i1).joint(i2-1).axis*jointVar(i2-1)) + strcmp(robot.kinematicChains(i1).joint(i2-1).type, 'prismatic')*eye(3);
            jointRot(:, :, i2) = jointRot(:, :, i2-1)*jointRrpy*jointRq;
            jointPxyz = robot.kinematicChains(i1).joint(i2).origin.xyz;
            jointPq = strcmp(robot.kinematicChains(i1).joint(i2).type, 'prismatic')*(robot.kinematicChains(i1).joint(i2).axis*jointVar(i2-1));            
            jointPos(:, :, i2) = jointPos(:, :, i2-1) + jointRot(:, :, i2-1)*jointRq*(jointPxyz + jointPq);
            jointPosJaco(:, :, i2) = jacobian(jointPos(:, :, i2), exJointVar);
            jointVel(:, :, i2) = jointPosJaco(:, :, i2)*dExJointVar;
            % Links
            linkRq = strcmp(robot.kinematicChains(i1).joint(i2).type, 'revolute')*Rpy2Mat(robot.kinematicChains(i1).joint(i2).axis*jointVar(i2)) + strcmp(robot.kinematicChains(i1).joint(i2).type, 'prismatic')*eye(3);
            linkRot(:, :, i2) = jointRot(:, :, i2)*Rpy2Mat(robot.kinematicChains(i1).link(i2).origin.rpy)*linkRq;            
            linkPos(:, :, i2) = jointPos(:, :, i2) + jointRot(:, :, i2)*Rpy2Mat(robot.kinematicChains(i1).link(i2).origin.rpy)*linkRq*robot.kinematicChains(i1).link(i2).origin.xyz;
            linkPosJaco(:, :, i2) = jacobian(linkPos(:, :, i2), exJointVar);            
            linkVel(:, :, i2) = linkPosJaco(:, :, i2)*dExJointVar;
        end
        tmpJointPosJaco = sym(zeros(size(jointPosJaco)));
        tmpLinkPosJaco = sym(zeros(size(linkPosJaco)));        
        tmpJointPosJaco(:, sum(nChainJoints(1:i1, 1))+1:sum(nChainJoints(1:(i1+1), 1)), :) = jointPosJaco;        
        tmpLinkPosJaco(:, sum(nChainJoints(1:i1, 1))+1:sum(nChainJoints(1:(i1+1), 1)), :) = linkPosJaco;        
        % Forward Kinematics
        robot.forwardKinematics.chain(i1).jointRot = (jointRot);
        robot.forwardKinematics.chain(i1).jointPos = (jointPos);
        robot.forwardKinematics.chain(i1).jointPosJaco = (tmpJointPosJaco);
        robot.forwardKinematics.chain(i1).jointVel = (jointVel);
        robot.forwardKinematics.chain(i1).linkRot = (linkRot);
        robot.forwardKinematics.chain(i1).linkPos = (linkPos);
        robot.forwardKinematics.chain(i1).linkPosJaco = (tmpLinkPosJaco);
        robot.forwardKinematics.chain(i1).linkVel = (linkVel);
        robot.forwardKinematics.chain(i1).symJointVar = jointVar;
        robot.forwardKinematics.chain(i1).symdJointVar = dJointVar;
    end
end