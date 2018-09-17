%*************************************************************************%
% Project: ROMOCOTOOL
% Name: mainRoModToolTest.m
% Type: matlab script
% Version: 1.0
% Description: This script is a RoModTool test sample 
% Author: Quoc-Viet DANG
%*************************************************************************%
restoredefaultpath;
clearvars; close all; clc

%% Addpath
roMoCoToolRootPath = fileparts(fileparts(fileparts([mfilename('fullpath'),'.m'])));
roModToollBuildPath = fullfile(roMoCoToolRootPath, 'roModTool', 'build');
addpath(roModToollBuildPath);

%% Physical parameters of the dynamical system
sysParams = getRoModToolSysParams();

%% System dynamics
nJoints = 2;
% Positions
positions = 0.1*ones(nJoints, 1);
% Velocities
velocities = 0.5*ones(nJoints, 1);
% State variable
states = [positions; velocities];
% Controls
controls = [0.3; 0];
% System dynamics
[M, C, G] = getSysDynMat(states, sysParams);
dStates = getStateSpaceModel(states, controls, [M, C, G]);
jointAccel = getForwardDynamics(states, controls, [M, C, G]);
jointTorq = getInverseDynamics(states, jointAccel, [M, C, G]);
%
errACC = jointAccel - dStates(nJoints+1:end, 1);
errCON = controls - full(jointTorq);
%
disp('******************************************************************');    
disp('Inertia matrix M:');
disp(full(M));
disp('******************************************************************');
disp('Coriolis matrix C:');
disp(full(C));
disp('******************************************************************');
disp('Gravity maxtrix G:')
disp(full(G));
disp('******************************************************************');
disp('Time-derivatives of state variable:')
disp(full(dStates));
disp('Accelerations of the joint variables and error:')
disp(full([jointAccel, errACC]));
disp('Controls and error:')
disp([full(jointTorq), errCON]);
