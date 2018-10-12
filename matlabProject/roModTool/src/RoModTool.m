%*************************************************************************%
%=========================================================================%
% Project: ROMOCOTOOL
% Name: RoModTool.m
% Type: matlab object
% Version: 1.0
% Description: This object class provides methods for the robot modelling
% Author: Quoc-Viet DANG
%=========================================================================%
classdef (Abstract) RoModTool < handle
    properties
        robot;             % symbolic class object        
    end
    
    methods(Abstract)
        %% Robot Modelling Tool using the Euler-Lagrange method via energy-based approach
        getEulerLagrangeRoModTool(obj)
        
        %% Robot Modelling Tool using the Newton-Euler method via torque/force balance approach
        getNewtonEulerRoModTool(obj)
    end
    
    
    methods
        function obj = RoModTool(robot)
            
            obj.robot = robot;
        end
    end
end
