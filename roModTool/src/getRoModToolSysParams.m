%*************************************************************************%
% Project: ROMOCOTOOL
% Name: getRoModToolSysParams.m
% Type: matlab function
% Version: 1.0
% Description: This function contains physical parameters used for 
%              the RoModTool test sample 
% Author: Quoc-Viet DANG
%*************************************************************************%
function sysParams = getRoModToolSysParams()
m1 = 1.5;               % [kg]
m2 = 1;                 % [kg]
l = 1.0;                % [m]
g = 9.81;               % [ms-2]

%% Physical parameters of the dynamical system
sysParams = [m1; m2; l; g];

end
