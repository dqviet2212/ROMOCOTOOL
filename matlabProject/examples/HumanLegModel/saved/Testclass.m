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

%% System kinematics and dynamics
fixedLegModel = FixedBaseRoModTool(robot);
% fixedLegModel.getRobotStructure();
% fixedLegModel.getForwardKinematics();
% fixedLegModel.getSymPhysicalParams();
% fixedLegModel.getSymStateVariable();
% fixedLegModel.getRobotEnergy();
% fixedLegModel.getSystemDynamics();


%% Symbolic matlab functions
matFuncRootPath = fullfile(roMoCoToolRootPath, 'examples', 'FixedLegModel', 'matfuncs');
if (exist(matFuncRootPath, 'dir') ~= 7)
    mkdir(matFuncRootPath);
end
fixedLegModel.getMatlabFunction(matFuncRootPath);

toc