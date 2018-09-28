function robot = getSymStateVariable(robot)
    nKinematicChains = size(robot.kinematicChains, 2);
    symJointVars = [];
    symdJointVars = [];
    for i1 = 1:nKinematicChains
        symJointVars = [symJointVars; robot.forwardKinematics.chain(i1).symJointVar]; %#ok<AGROW>
        symdJointVars = [symdJointVars; robot.forwardKinematics.chain(i1).symdJointVar]; %#ok<AGROW>
    end
    if strcmp(robot.base.type, 'fixed')
        robot.symStateVariable = [symJointVars; symdJointVars];
    else
        robot.symStateVariable = [robot.base.symJointVar; symJointVars; robot.base.symdJointVar; symdJointVars];
    end    
end