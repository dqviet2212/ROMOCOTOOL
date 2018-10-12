restoredefaultpath; clearvars; close all; clc;

%% Test function: rotM = Rpy2Mat(rpy)
r = sym('r');
p = sym('p');
y = sym('y');
rpy = [r; p; y];
rotM = Rpy2Mat(rpy);
disp('Rpy2Mat -OK');
disp('==================================================================');
disp(rotM);

%% Test function: rpy = Mat2Rpy(rotM)
rpy1 = deg2rad([-180; -90; +180]);
rotM1 = Rpy2Mat(rpy1);
rpy2 = Mat2Rpy(rotM1);
err = rpy1 - rpy2;
disp('Mat2Rpy -OK');
disp('==================================================================');
disp(err);

%% Test function: robotStr = getRobotStructure(L1, L2, L3, l1, l2, l3, m1, m2, m3)
L1 = sym('L1');
L2 = sym('L2');
L3 = sym('L3');
l1 = sym('l1');
l2 = sym('l2');
l3 = sym('l3');
m1 = sym('m1');
m2 = sym('m2');
m3 = sym('m3');
%
robotStr = getRobotStructure(L1, L2, L3, l1, l2, l3, m1, m2, m3);
disp('getRobotStructure -OK');
disp('==================================================================');
disp(robotStr);

%% Test function: forwardKinematics = getForwardKinematics(robotStr, nJoints)
nJoints = 3;
forwardKinematics = getForwardKinematics(robotStr, nJoints);
disp('getForwardKinematics -OK');
disp('==================================================================');
disp(forwardKinematics);
% Joints
forwardKinematics.jointRot(:, :, 1)
forwardKinematics.jointRot(:, :, 2)
forwardKinematics.jointRot(:, :, 3)
forwardKinematics.jointPos(:, :, 1)
forwardKinematics.jointPos(:, :, 2)
forwardKinematics.jointPos(:, :, 3)
% Links
forwardKinematics.linkRot(:, :, 1)
forwardKinematics.linkRot(:, :, 2)
forwardKinematics.linkRot(:, :, 3)
forwardKinematics.linkPos(:, :, 1)
forwardKinematics.linkPos(:, :, 2)
forwardKinematics.linkPos(:, :, 3)
