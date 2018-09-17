%*************************************************************************%
% Project: ROMOCOTOOL
% Name: mainCartPoleModel.m
% Type: matlab script
% Version: 1.0
% Description: This script is a sample how to use the modelling tool
%              RoModTool of ROMOCOTOOL project
% Author: Quoc-Viet DANG                                                  
%*************************************************************************%
restoredefaultpath;
clearvars; close all; clc
tic

%% Addpath
roMoCoToolRootPath = fileparts(fileparts(fileparts(fileparts([mfilename('fullpath'),'.m']))));
roModToolRootPath = fullfile(roMoCoToolRootPath, 'src');
addpath(roModToolRootPath);

%% Symbolic variables declaration
syms m1 m2 l g
syms q1 q2 dq1 dq2 d2q1 d2q2
syms u1 u2

%% Kinematic and potential energy
% Kinematic energy
Ek = simplify(0.5*(m1+m2)*dq1^2 + 0.5*m1*l^2*dq2^2 + m1*l*dq1*dq2*cos(q2));
% Potentialenergy
Ep = simplify(m1*g*l*cos(q2));
% Positions
positions = [q1; q2];
% Velocities
velocities = [dq1; dq2];
% Accelerations
accelerations = [d2q1; d2q2];
% State variable
states = [positions; velocities];
% Controls
controls = [u1; 0];
% System parameters
sysParams = [m1; m2; l; g];

%% System dynamics model
roModTool = RoModTool(Ek, Ep, states, accelerations, controls, sysParams);
sysDyn = roModTool.getSystemDynamics();
matFuncRootPath = fullfile(fileparts(fileparts([mfilename('fullpath'),'.m'])), 'build');
if (exist(matFuncRootPath, 'dir') ~= 7)
    mkdir(matFuncRootPath);
end
[sysDynMatPath, ssModPath, sysIDPath, sysFDPath] = roModTool.sysDyn2MatFunc(matFuncRootPath, sysDyn);

%% Results
nStates = sysDyn.nStates;
nControls = sysDyn.nControls;
activeConInd = sysDyn.activeConInd;
passiveConInd = sysDyn.passiveConInd;
M = sysDyn.M;
C = sysDyn.C;
G = sysDyn.G;
%
disp('******************************************************************');    
fprintf('Number of state variable: nStates = %d\n', nStates);
disp('******************************************************************');    
fprintf('Number of controls: nControls = %d\n', nControls);
disp('******************************************************************');    
fprintf(['Active controls index: activeConInd = ', regexprep(mat2str(ones(1, max(length(activeConInd), 1))), '1', '%d'), '\n'], activeConInd);
disp('******************************************************************');    
fprintf(['Passive controls index: passiveConInd = ', regexprep(mat2str(ones(1, max(length(passiveConInd), 1))), '1', '%d'), '\n'], passiveConInd);
disp('******************************************************************');    
disp('Inertia matrix M:');
disp(M);
disp('******************************************************************');
disp('Coriolis matrix C:');
disp(C);
disp('******************************************************************');
disp('Garity maxtrix G:')
disp(G);
disp('******************************************************************');
disp('Root paths to system dynamics:')
fprintf('%s\n%s\n%s\n%s\n', sysDynMatPath, ssModPath, sysIDPath, sysFDPath);

toc
return;