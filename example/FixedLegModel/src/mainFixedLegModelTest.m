%*************************************************************************%
% Project: ROMOCOTOOL
% Name: mainFixedLegModel.m
% Type: matlab script
% Version: 1.0
% Description: This script is a test sample 
% Author: Quoc-Viet DANG
%*************************************************************************%
restoredefaultpath;
clearvars; close all; clc

%% Addpath
roMoCoToolRootPath = fileparts(fileparts(fileparts(fileparts([mfilename('fullpath'),'.m']))));
examRootPath = fullfile(roMoCoToolRootPath, 'example');
fixedLegModelRootPath = fullfile(examRootPath, 'FixedLegModel');
fixedLegModelBuildPath = fullfile(fixedLegModelRootPath, 'build');
addpath(fixedLegModelBuildPath);

%% Physical parameters of the dynamical system
sysParams = getSysParams();

%% System dynamics
nJoints = 3;
% Positions
positions = 0.1*ones(nJoints, 1);
% Velocities
velocities = 0.5*ones(nJoints, 1);
% Accelerations
accelerations = 0.8*ones(nJoints, 1);
% State variable
states = [positions; velocities];
% Controls
controls = 0.5*ones(nJoints, 1);
% System dynamics
[M, C, G] = getSysDynMat(states, sysParams);
dStates = getSysDynamics(states, controls, [M, C, G]);
d2q = getSysDyn2FD(states, controls, [M, C, G]);
torques = getSysDyn2ID(states, accelerations, [M, C, G])';
%
disp('******************************************************************');    
disp('Inertia matrix M:');
disp(M);
disp('******************************************************************');
disp('Coriolis matrix C:');
disp(C);
disp('******************************************************************');
disp('Gravity maxtrix G:')
disp(G);
disp('******************************************************************');
disp('Time-derivatives of state variable:')
disp(dStates);
disp(d2q);
disp(torques);
