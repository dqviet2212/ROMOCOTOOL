%*************************************************************************%
% Project: ROMOCOTOOL
% Name: mainFixedBaseLegModel.m
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
robotName = 'FixedBaseHumanLegModel';
robot = getKinematicChainDef(robotName);

%% System kinematics and dynamics
fixedBaseLegModel = FixedBaseRoModTool(robot);

%% Symbolic matlab functions
matFuncRootPath = fullfile(roMoCoToolRootPath, 'examples', 'HumanLegModel', 'matfuncs', 'fixedbase');
if (exist(matFuncRootPath, 'dir') ~= 7)
    mkdir(matFuncRootPath);
end
simplifiedCode = false;
sparseMatrix = false;
fixedBaseLegModel.getEulerLagrangeRoModTool(matFuncRootPath, simplifiedCode, sparseMatrix);

%% Test with FixedLegModel
addpath(matFuncRootPath);
% Symbolic physical parameters
g = sym('g');
L1 = sym('L1');
L2 = sym('L2');
L3 = sym('L2');
l1 = sym('l1');
l2 = sym('l2');
l3 = sym('l3');
m1 = sym('m1');
m2 = sym('m2');
m3 = sym('m3');
symFixedBaseLegModelParams = getFixedBaseLegModelPhysicalParams(g, L1, L2, L3, l1, l2, l3, m1, m2, m3);
% Symbolic state variable
q1 = sym('q1');
q2 = sym('q2');
q3 = sym('q3');
dq1 = sym('dq1');
dq2 = sym('dq2');
dq3 = sym('dq3');
symStateVariable = [q1; q2; q3; dq1; dq2; dq3];
% Symbolic control 
u1 = sym('u1');
u2 = sym('u2');
u3 = sym('u3');
symControls = [u1; u2; u3];
%
joint1Pos = getJoint11Position(symFixedBaseLegModelParams, symStateVariable);
link1Pos = getLink11Position(symFixedBaseLegModelParams, symStateVariable);
joint2Pos = getJoint12Position(symFixedBaseLegModelParams, symStateVariable);
link2Pos = getLink12Position(symFixedBaseLegModelParams, symStateVariable);
joint3Pos = getJoint13Position(symFixedBaseLegModelParams, symStateVariable);
link3Pos = getLink13Position(symFixedBaseLegModelParams, symStateVariable);
%
disp('===================R1=========================');
disp(simplify([joint1Pos, link1Pos]));
disp('===================R2=========================');
disp(simplify([joint2Pos, link2Pos]));
disp('===================R3=========================');
disp(simplify([joint3Pos, link3Pos]));
%
M = getInertiaMatrix(symFixedBaseLegModelParams, symStateVariable);
C = getCoriolisMatrix(symFixedBaseLegModelParams, symStateVariable);
G = getGravityTorque(symFixedBaseLegModelParams, symStateVariable);
jointAccel = getForwardDynamics([M, C, G], symStateVariable, symControls);
dStates = getStateSpaceModel([M, C, G], symStateVariable, symControls);

%% Numerical test 
g = 9.81;
L1 = 0.403;
L2 = 0.380;
L3 = 0.415;
l1 = 0.5*L1;
l2 = 0.5*L2;
l3 = 0.5*L3;
m1 = 0.5;
m2 = 0.6;
m3 = 0.25;
numStateVar = rand(6, 1);
numCon = rand(3, 1);
numFixedBaseLegModelParams = getFixedBaseLegModelPhysicalParams(g, L1, L2, L3, l1, l2, l3, m1, m2, m3);
M = getInertiaMatrix(numFixedBaseLegModelParams, numStateVar)
C = getCoriolisMatrix(numFixedBaseLegModelParams, numStateVar)
G = getGravityTorque(numFixedBaseLegModelParams, numStateVar)
jointAccel = getForwardDynamics([M, C, G], numStateVar, numCon)
dStates = getStateSpaceModel([M, C, G], numStateVar, numCon)





