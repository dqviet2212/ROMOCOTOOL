%*************************************************************************%
%=========================================================================%
% Project: ROMOCOTOOL
% Name: TrajectoryOpt.m
% Type: matlab object
% Version: 1.0
% Description: This object class is an implementation of the class trajOptTool
% Author: Quoc-Viet DANG
%=========================================================================%
classdef TrajectoryOpt < trajOptTool
    
    properties
       
    end
    
    methods
        %% Constructor        
        %=================================================================%
        function obj = TrajectoryOpt(params0, Aineq, bineq, Aeq, beq, lb, ub,...
                                     weightTable, nlIneqSelectionTable, nlEqSelectionTable,...
                                     optProbObj)
            
            obj = obj@trajOptTool(params0, Aineq, bineq, Aeq, beq, lb, ub,...
                                weightTable, nlIneqSelectionTable, nlEqSelectionTable,...
                                optProbObj);
            
            % costTable
            obj.costTable{1} = @obj.computeCost;
                        
            % nlIneqConstraintTable
            obj.nlIneqConstraintTable{1} = @obj.computeNLIneqConstraint;
            
            % nlEqConstraintTable
            obj.nlEqConstraintTable{1} = @obj.computeNLEqConstraint;
        end
        
        %% computeSharedInformation
        % This function computes all necessary quantities and puts them 
        % in a data structure named: sharedInformation
        %=================================================================%
        function computeSharedInformation(obj, params)      
            %% Initialization
            optProbObj = obj.optProbObj;
            
            %% Optimization parameters         
            sysDynamics = optParams2sysDynamics(params, optProbObj);
                        
            %% Constraints at collocation nodes
            colEqCst = getCollocationConstraints(sysDynamics, optProbObj);
            
            %% Objective            
            cost = getOptObjective(sysDynamics, optProbObj);
            
            %% Equality constraints
            stepEqCst = getStepEqualityConstraints(sysDynamics, optProbObj);
            pathEqCst = getPathEqualityConstraints(sysDynamics, optProbObj);
            
            %% Inequality constraints
            stepIneqCst = getStepInequalityConstraints(sysDynamics, optProbObj);
            pathIneqCst = getPathInequalityConstraints(sysDynamics, optProbObj);
            
            %% Save data to sharedInformation object
            obj.sharedInformation.cost            = cost;
            obj.sharedInformation.stepIneqCst     = stepIneqCst;
            obj.sharedInformation.pathIneqCst     = pathIneqCst;
            obj.sharedInformation.stepEqCst       = stepEqCst;
            obj.sharedInformation.pathEqCst       = [colEqCst; pathEqCst];
%*************************************************************************%   
        end
        
        %% costTable
        % This section computes all costs returned in a real value
        %=================================================================%
        % 1
        function cost = computeCost(obj)
            cost = obj.sharedInformation.cost;
        end
        
        %% nlIneqConstraintTable
        % This section computes all nonlinear inequality constraints 
        % returned in a vector
        %=================================================================%        
        function nonlineqConstraint = computeNLIneqConstraint(obj)
            nonlineqConstraint = [obj.sharedInformation.stepIneqCst;
                                  obj.sharedInformation.pathIneqCst;
                                 ];
        end
        
        %% nlEqConstraintTable
        % This section computes all nonlinear equality constraints 
        % returned in a vector
        %=================================================================%
        function nonleqConstraint = computeNLEqConstraint(obj)
            nonleqConstraint = [obj.sharedInformation.stepEqCst;
                                obj.sharedInformation.pathEqCst;
                               ];
        end
    end
end
%=========================================================================%
%============================ END OF PROGRAM =============================%
%=========================================================================%