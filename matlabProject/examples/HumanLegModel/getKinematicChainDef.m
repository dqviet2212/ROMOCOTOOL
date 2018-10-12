function robot = getKinematicChainDef(robotName)
    %% Robot name
    robot.name = robotName;
    
    %% Base type
    % 'fixed': if the base is fixed to the world coordinate frame
    % 'floating': if the base is moved to the world coordinate frame
    robot.base.type = 'fixed';
    % robot.base.type = 'floating';
                
    %% Kinematic chain 1
    robot.kinematicChains(1).joint(1).axis = [0; 1; 0]; robot.kinematicChains(1).joint(1).type = 'revolute';
    robot.kinematicChains(1).joint(2).axis = [0; 1; 0]; robot.kinematicChains(1).joint(2).type = 'revolute';
    robot.kinematicChains(1).joint(3).axis = [0; 1; 0]; robot.kinematicChains(1).joint(3).type = 'revolute';
    
    %% Kinematic chain 2
%     robot.kinematicChains(2).joint(1).axis = [0; 1; 0]; robot.kinematicChains(2).joint(1).type = 'revolute';
%     robot.kinematicChains(2).joint(2).axis = [0; 1; 0]; robot.kinematicChains(2).joint(2).type = 'revolute';
%     robot.kinematicChains(2).joint(3).axis = [0; 1; 0]; robot.kinematicChains(2).joint(3).type = 'revolute';
%     robot.kinematicChains(2).joint(4).axis = [0; 1; 0]; robot.kinematicChains(2).joint(4).type = 'revolute';
%     robot.kinematicChains(2).joint(5).axis = [0; 1; 0]; robot.kinematicChains(2).joint(5).type = 'revolute';
%     robot.kinematicChains(2).joint(6).axis = [0; 1; 0]; robot.kinematicChains(2).joint(6).type = 'revolute';
%     robot.kinematicChains(2).joint(7).axis = [0; 1; 0]; robot.kinematicChains(2).joint(7).type = 'revolute';
%     robot.kinematicChains(2).joint(8).axis = [0; 1; 0]; robot.kinematicChains(2).joint(8).type = 'revolute';
%     robot.kinematicChains(2).joint(9).axis = [0; 1; 0]; robot.kinematicChains(2).joint(9).type = 'revolute';
%     robot.kinematicChains(2).joint(10).axis = [0; 1; 0]; robot.kinematicChains(2).joint(10).type = 'revolute';
%     robot.kinematicChains(2).joint(11).axis = [0; 1; 0]; robot.kinematicChains(2).joint(11).type = 'revolute';
%     robot.kinematicChains(2).joint(12).axis = [0; 1; 0]; robot.kinematicChains(2).joint(12).type = 'revolute';
    
end
