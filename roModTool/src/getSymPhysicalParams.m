function symPhysicalParams = getSymPhysicalParams(robot)
    symBasePhysicalParams = robot.base.symPhysicalParams;
    nKinematicChains = size(robot.kinematicChains, 2);   
    symJointPhysicalParams = [];
    symLinkPhysicalParams = [];
    for i1 = 1:nKinematicChains
        nJoints = size(robot.kinematicChains(i1).joint, 2);
        for i2 = 1:nJoints
            symJointPhysicalParams = [symJointPhysicalParams; robot.kinematicChains(i1).joint(i2).symPhysicalParams]; %#ok<AGROW>
            symLinkPhysicalParams = [symLinkPhysicalParams; robot.kinematicChains(i1).link(i2).symPhysicalParams]; %#ok<AGROW>
        end
    end    
    symPhysicalParams = [symBasePhysicalParams; symJointPhysicalParams; symLinkPhysicalParams];
end