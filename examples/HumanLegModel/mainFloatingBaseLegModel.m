%*************************************************************************%
% Project: ROMOCOTOOL
% Name: mainFloatingBaseLegModel.m
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
robotName = 'FloatingBaseHumanLegModel';
robot = getKinematicChainDef(robotName);

%% System kinematics and dynamics
floatingBaseLegModel = FloatingBaseRoModTool(robot);

%% Symbolic matlab functions
matFuncRootPath = fullfile(roMoCoToolRootPath, 'examples', 'HumanLegModel', 'matfuncs', 'floatingbase');
if (exist(matFuncRootPath, 'dir') ~= 7)
    mkdir(matFuncRootPath);
end
simplifiedCode = false;
sparseMatrix = false;
floatingBaseLegModel.getEulerLagrangeRoModTool(matFuncRootPath, simplifiedCode, sparseMatrix);

%
toc