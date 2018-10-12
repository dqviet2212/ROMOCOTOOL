clearvars; close all; clc;

%% Robot structure definition
tic
robotName = 'FixedLegModel';
robot = getRobotStructure(robotName);
robot = getForwardKinematics(robot);
toc

return;
j1x = sym('j1x');
j1y = sym('j1y');
j1z = sym('j1z');
j1r = sym('j1r');
j1p = sym('j1p');
j1ya = sym('j1y');
l1x = sym('l1x');
l1y = sym('l1x');
l1z = sym('l1x');

% matlabFunction(forwardKinematics.linkPos(1, 1, 1), 'File', 'Vars', [j1x; j1y; j1z; j1r; j1p; j1a; l1x; l1y; l1z], 'Outputs', {'link1Pos'}, 'Sparse', true);


return;
%%
nJoints = 12;
for i1 = 1:nJoints
    joint(i1).name = ['joint', num2str(i1)];
    joint(i1).axis = str2sym([['a', num2str(i1), 'x']; ['a', num2str(i1), 'y']; ['a', num2str(i1), 'z']]);
    joint(i1).origin.xyz = str2sym([['j', num2str(i1), 'x']; ['j', num2str(i1), 'y']; ['j', num2str(i1), 'z']]);
    joint(i1).origin.rpy = str2sym([['j', num2str(i1), 'r']; ['j', num2str(i1), 'p']; ['j', num2str(i1), 'y']]);
    joint(i1).parentlink = ['link', num2str(i1-1)];
    joint(i1).childlink = ['link', num2str(i1)];
    %
    link(i1).name = ['link', num2str(i1)];
    link(i1).origin.xyz = str2sym([['l', num2str(i1), 'x']; ['l', num2str(i1), 'y']; ['l', num2str(i1), 'z']]);
    link(i1).origin.rpy = str2sym([['l', num2str(i1), 'r']; ['l', num2str(i1), 'p']; ['l', num2str(i1), 'y']]);
    link(i1).mass = str2sym(['m', num2str(i1)]);
    link(i1).inertia(1, 1) = str2sym(['ixx', num2str(i1)]);
    link(i1).inertia(1, 2) = str2sym(['ixy', num2str(i1)]);
    link(i1).inertia(1, 3) = str2sym(['ixz', num2str(i1)]);
    link(i1).inertia(2, 1) = str2sym(['ixy', num2str(i1)]);
    link(i1).inertia(2, 2) = str2sym(['iyy', num2str(i1)]);
    link(i1).inertia(2, 3) = str2sym(['iyz', num2str(i1)]);
    link(i1).inertia(3, 1) = str2sym(['ixz', num2str(i1)]);
    link(i1).inertia(3, 2) = str2sym(['iyz', num2str(i1)]);
    link(i1).inertia(3, 3) = str2sym(['izz', num2str(i1)]);
end

