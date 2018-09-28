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
        getRobotStructure(obj)
        getForwardKinematics(obj)
        getSymPhysicalParams(obj)
        getSymStateVariable(obj)
        getRobotEnergy(obj)        
        getSystemDynamics(obj)        
        getMatlabFunction(obj, matFuncRootPath)
    end
    
    
    methods
        function obj = RoModTool(robot)
            
            obj.robot = robot;
        end
    end
end
