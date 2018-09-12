%*************************************************************************%
% Author: Quoc-Viet DANG                                                  
% Project: optConTools
% Name: mainFixedLegModel.m
% Type: matlab script
% Version: 08 September 2018                                              
% Description:
%*************************************************************************%
restoredefaultpath;
clearvars; close all; clc

tic
%% Addpath
optConToolsRootPath = fileparts(fileparts(fileparts([mfilename('fullpath'),'.m'])));
modToolsRootPath = fullfile(optConToolsRootPath, 'src', 'modTools');
addpath(modToolsRootPath);

%% Symbolic variables declaration
syms g
syms m1 l1 L1    % Thigh
syms m2 l2 L2    % Shank
syms m3 l3 L3    % Foot
syms q1 dq1
syms q2 dq2
syms q3 dq3
syms u1 u2 u3

%% Kinematic and potential energy
% Thigh
J1 = (1/12)*m1*L1^2 + m1*(0.5*L1 - l1)^2;
x1 = @(q1)(l1*sin(q1));
z1 = @(q1)(l1*cos(q1));
dx1 = diff(x1, q1)*dq1;
dz1 = diff(z1, q1)*dq1;
v1 = dx1^2 + dz1^2;
h1 = l1*cos(q1);
Ek1 = 0.5*m1*v1^2 + 0.5*J1*dq1^2;
Ep1 = m1*g*h1;
% Shank
J2 = (1/12)*m2*L2^2 + m2*(0.5*L2 - l2)^2;
x2 = @(q1, q2)(L1*sin(q1) + l2*sin(q1 + q2));
z2 = @(q1, q2)(L1*cos(q1) + l2*cos(q1 + q2));
dx2 = diff(x2, q1)*dq1 + diff(x2, q2)*dq2;
dz2 = diff(z2, q1)*dq1 + diff(z2, q2)*dq2;
v2 = dx2^2 + dz2^2;
h2 = L1*cos(q1) + l2*cos(q1 + q2);
Ek2 = 0.5*m2*v2^2 + 0.5*J2*dq2^2;
Ep2 = m2*g*h2;
% Foot
J3 = (1/12)*m3*L3^2 + m3*(0.5*L3 - l3)^2;
x3 = @(q1, q2, q3)(L1*sin(q1) + L2*sin(q1 + q2) + l3*cos(q1 + q2 +q3));
z3 = @(q1, q2, q3)(L1*cos(q1) + L2*cos(q1 + q2) - l3*sin(q1 + q2 + q3));
dx3 = diff(x3, q1)*dq1 + diff(x3, q2)*dq2 + diff(x3, q3)*dq3;
dz3 = diff(z3, q1)*dq1 + diff(z3, q2)*dq2 + diff(z3, q3)*dq3;
v3 = dx3^2 + dz3^2;
h3 = L1*cos(q1) + L2*cos(q1 + q2) - L3*sin(q1 + q2 + q3);
Ek3 = 0.5*m3*v3^2 + 0.5*J3*dq3^2;
Ep3 = m3*g*h3;
% Kinematic energy
Ek = Ek1 + Ek2 + Ek3;
% Potentialenergy
Ep = Ep1 + Ep2 + Ep3;
% Positions
positions = [q1; q2; q3];
% Velocities
velocities = [dq1; dq2; dq3];
% State variable
states = [positions; velocities];
% Controls
controls = [u1; u2; u3];
% System parameters
sysParams = [m1; l1; L1; m2; l2; L2; m3; l3; L3; g];

%% System dynamics model
optConToolsModObj = OptConToolsMod(Ek, Ep, states, controls, sysParams);
sysDynamicsMod = optConToolsModObj.getSysDynMod();
matFuncRootPath = fullfile(optConToolsRootPath, 'example', 'FixedLegModel');
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

toc
return;