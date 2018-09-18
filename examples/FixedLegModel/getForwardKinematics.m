function forwardKinematics = getForwardKinematics(robotStr, nJoints)
    %% Base
    % Position of the base in the world coordinate frame
    basePos = robotStr.base.origin.xyz;
    % Rotation matrix of the base in the world coordinate frame    
    baseRot = Rpy2Mat(robotStr.base.origin.rpy);
    
    %% Joints and Links
    jointRot = sym(zeros(3, 3, nJoints));
    jointPos = sym(zeros(3, 1, nJoints));
    jointVel = sym(zeros(3, 1, nJoints));
    linkRot = sym(zeros(3, 3, nJoints));
    linkPos = sym(zeros(3, 1, nJoints));
    linkVel = sym(zeros(3, 1, nJoints));
    jointVar = sym('q', [nJoints, 1]);
    dJointVar = sym('dq', [nJoints, 1]);
    dBasePos = str2sym(['dpB1'; 'dpB2'; 'dpB3']);
    dBaseOri = str2sym(['drB1'; 'drB2'; 'drB3']);
    
    %% Joint1    
    jointRot(:, :, 1) = baseRot*Rpy2Mat(robotStr.joint(1).origin.rpy);
    jointPos(:, :, 1) = basePos + baseRot*robotStr.joint(1).origin.xyz;
    jointVel(:, :, 1) = jacobian(jointPos(:, :, 1), [robotStr.base.origin.xyz; robotStr.base.origin.rpy; jointVar])*[dBasePos; dBaseOri; dJointVar];
    
    %% Link1
    link1Rq = strcmp(robotStr.joint(1).type, 'revolute')*Rpy2Mat(robotStr.joint(1).axis*jointVar(1)) + strcmp(robotStr.joint(1).type, 'prismatic')*eye(3);
    linkRot(:, :, 1) = jointRot(:, :, 1)*Rpy2Mat(robotStr.link(1).origin.rpy)*link1Rq;
    linkPos(:, :, 1) = jointPos(:, :, 1) + jointRot(:, :, 1)*link1Rq*robotStr.link(1).origin.xyz;
    linkVel(:, :, 1) = jacobian(linkPos(:, :, 1), [robotStr.base.origin.xyz; robotStr.base.origin.rpy; jointVar])*[dBasePos; dBaseOri; dJointVar];
    
    %% Other Joints and Links
    for i1 = 2:nJoints
        % Joints
        jointRrpy = Rpy2Mat(robotStr.joint(i1-1).origin.rpy);
        jointRq = strcmp(robotStr.joint(i1-1).type, 'revolute')*Rpy2Mat(robotStr.joint(i1-1).axis*jointVar(i1-1)) + strcmp(robotStr.joint(i1-1).type, 'prismatic')*eye(3);        
        jointRot(:, :, i1) = simplify(jointRot(:, :, i1-1)*jointRrpy*jointRq);
        jointPxyz = robotStr.joint(i1).origin.xyz;
        jointPq = strcmp(robotStr.joint(i1).type, 'prismatic')*(robotStr.joint(i1).axis*jointVar(i1-1));
        jointPos(:, :, i1) = simplify(jointPos(:, :, i1-1) + jointRot(:, :, i1-1)*jointRq*(jointPxyz + jointPq));
        jointVel(:, :, i1) = jacobian(jointPos(:, :, i1), [robotStr.base.origin.xyz; robotStr.base.origin.rpy; jointVar])*[dBasePos; dBaseOri; dJointVar];
        % Links
        linkRq = strcmp(robotStr.joint(i1).type, 'revolute')*Rpy2Mat(robotStr.joint(i1).axis*jointVar(i1)) + strcmp(robotStr.joint(i1).type, 'prismatic')*eye(3);
        linkRot(:, :, i1) = simplify(jointRot(:, :, i1)*Rpy2Mat(robotStr.link(i1).origin.rpy)*linkRq);
        linkPos(:, :, i1) = simplify(jointPos(:, :, i1) + jointRot(:, :, i1)*Rpy2Mat(robotStr.link(i1).origin.rpy)*linkRq*robotStr.link(i1).origin.xyz);
        linkVel(:, :, i1) = jacobian(linkPos(:, :, i1), [robotStr.base.origin.xyz; robotStr.base.origin.rpy; jointVar])*[dBasePos; dBaseOri; dJointVar];
    end    
    
    %% forwardKinematics
    forwardKinematics.jointRot = jointRot;
    forwardKinematics.jointPos = jointPos;
    forwardKinematics.jointVel = jointVel;
    forwardKinematics.linkRot = linkRot;
    forwardKinematics.linkPos = linkPos;
    forwardKinematics.linkVel = linkVel;
end