function [jointAxes, jointTypes] = getJointAxesDef()
    joint1Axis = [0; 1; 0]; joint1Type = 'revolute';
    joint2Axis = [0; 1; 0]; joint2Type = 'revolute';
    joint3Axis = [0; 1; 0]; joint3Type = 'revolute';    
    %
    jointAxes = [joint1Axis, joint2Axis, joint3Axis];
    jointTypes = {joint1Type, joint2Type, joint3Type};
end
