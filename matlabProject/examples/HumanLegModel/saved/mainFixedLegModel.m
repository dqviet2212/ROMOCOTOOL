%*************************************************************************%
% Project: ROMOCOTOOL
% Name: mainFixedLegModel.m
% Type: matlab script
% Version: 1.0
% Description: This script is a sample how to use the modelling tool
%              RoModTool of ROMOCOTOOL project
% Author: Quoc-Viet DANG
%*************************************************************************%
restoredefaultpath;
clearvars; close all; clc

%% Addpath
roMoCoToolRootPath = fileparts(fileparts(fileparts([mfilename('fullpath'),'.m'])));
roModToolRootPath = fullfile(roMoCoToolRootPath, 'roModTool', 'src');
addpath(roModToolRootPath);

%% Robot structure definition
tic
robotName = 'FixedLegModel';
robot = getKinematicChainDef(robotName);
robot = getRobotStructure(robot);
robot = getForwardKinematics(robot);
robot = getRobotEnergy(robot);
robot = getSymStateVariable(robot);
robot = getSystemDynamics(robot);

return;
symPhysicalParams = getSymPhysicalParams(robot);
symStates = getSymStateVariable(robot);

%% Symbolic matlab functions
matFuncRootPath = fullfile(roMoCoToolRootPath, 'examples', 'FixedLegModel', 'matfuncs');
if (exist(matFuncRootPath, 'dir') ~= 7)
    mkdir(matFuncRootPath);
end
%
nKinematicChains = size(robot.kinematicChains, 2);
nFuncs = 0;
for i1 = 1:nKinematicChains
    nJoints = size(robot.kinematicChains(i1).joint, 2);
    for i2 = 1:nJoints
        nFuncs = nFuncs + 1;
        jointMatFuncName = ['getJoint', num2str(i1), num2str(i2), 'Position'];
        jointOutputVars = robot.forwardKinematics.chain(i1).jointPos(:, :, i2);
        jointSymMatFuncPath{nFuncs} = getMatlabFunction(matFuncRootPath, jointMatFuncName, jointOutputVars, symPhysicalParams, symStates); %#ok<SAGROW>
        linkMatFuncName = ['getLink', num2str(i1), num2str(i2), 'Position'];
        linkOutputVars = robot.forwardKinematics.chain(i1).linkPos(:, :, i2);
        linkSymMatFuncPath{nFuncs} = getMatlabFunction(matFuncRootPath, linkMatFuncName, linkOutputVars, symPhysicalParams, symStates); %#ok<SAGROW>
    end
end
toc

%% Test with FixedLegModel
addpath(matFuncRootPath);
%
L1 = sym('L1');
L2 = sym('L2');
L3 = sym('L2');
l1 = sym('l1');
l2 = sym('l2');
l3 = sym('l3');
m1 = sym('m1');
m2 = sym('m2');
m3 = sym('m3');
%
symFixedLegModelParams = getSymFixedLegModelSymParams(L1, L2, L3, l1, l2, l3, m1, m2, m3);
%
joint1Pos = getJoint11Position(symFixedLegModelParams, symStates);
link1Pos = getLink11Position(symFixedLegModelParams, symStates);
joint2Pos = getJoint12Position(symFixedLegModelParams, symStates);
link2Pos = getLink12Position(symFixedLegModelParams, symStates);
joint3Pos = getJoint13Position(symFixedLegModelParams, symStates);
link3Pos = getLink13Position(symFixedLegModelParams, symStates);
%
disp('===================R1=========================');
disp(simplify([joint1Pos, link1Pos]));
disp('===================R2=========================');
disp(simplify([joint2Pos, link2Pos]));
disp('===================R3=========================');
disp(simplify([joint3Pos, link3Pos]));

