%*************************************************************************%
% Author: Quoc-Viet DANG                                                  
% Project: optConTools
% Name: testOptConToolsMod.m
% Type: matlab script
% Version: 08 September 2018                                              
% Description:
%*************************************************************************%
restoredefaultpath;
clearvars; close all; clc

%% Symbolic variables declaration
syms m1 m2 l g
syms q1 q2 dq1 dq2
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
% State variable
states = [positions; velocities];
% Controls
controls = [u1; 0];
% System parameters
sysParams = [m1; m2; l; g];

%% System dynamics model
optConToolsModObj = OptConToolsMod(Ek, Ep, states, controls, sysParams);
sysDynamicsMod = optConToolsModObj.getSysDynMod();
matFuncRootPath = '/media/dqviet/DATA/research/optConTools/build';
matFunctionPath = optConToolsModObj.getSysDynMod2MatFunc(matFuncRootPath, sysDynamicsMod);

%% Results
nStates = sysDynamicsMod.nStates;
nControls = sysDynamicsMod.nControls;
activeConInd = sysDynamicsMod.activeConInd;
passiveConInd = sysDynamicsMod.passiveConInd;
M = sysDynamicsMod.M;
C = sysDynamicsMod.C;
G = sysDynamicsMod.G;
EoM = sysDynamicsMod.EoM;
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
disp('Equation of motion:')
disp(EoM);

return;