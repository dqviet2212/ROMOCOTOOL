%*************************************************************************%
% Project: ROMOCOTOOL
% Name: getFixedLegModelSysParams.m
% Type: matlab function
% Version: 1.0
% Description: This function contains physical parameters used for 
%              the FixedLegModel test sample 
% Author: Quoc-Viet DANG
%*************************************************************************%
function sysParams = getFixedLegModelSysParams()
%% Gravity
g = 9.81;   % [ms-2]

%% Thigh
m1 = 1.5;               % [kg]
L1 = 0.45;              % [m]
l1 = 0.4*L1;

%% Shank
m2 = 1;                 % [kg]
L2 = 0.403;             % [m]
l2 = 0.55*L2;

%% Foot
m3 = 0.4;               % [kg]
L3 = 0.20;              % [m]
l3 = 0.4*L3;

%% Physical parameters of the dynamical system
sysParams = [m1; l1; L1; m2; l2; L2; m3; l3; L3; g];

end
