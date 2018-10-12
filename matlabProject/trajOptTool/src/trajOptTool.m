%*************************************************************************%
%=========================================================================%
% Project: ROMOCOTOOL
% Name: trajOptTool.m
% Type: matlab class
% Version: 1.0
% Description: This object class provides methods for the trajectory
% optimization
% Author: Quoc-Viet DANG
%=========================================================================%


classdef (Abstract) trajOptTool < handle    
    %% Abstract Properties
    properties
        params0;                % @params: params0 - initial condition of optimized parameters
        Aineq;                  % @params: Aineq - coefficient matrix of linear inequality constraint
        bineq;                  % @params: bineq - bound vector of linear inequality constraint
        Aeq;                    % @params: Aeq - coefficient matrix of linear equality constraint
        beq;                    % @params: beq - bound vector of linear equality constraint
        lb;                     % @params: lb - lower bound vector of optimized parameters
        ub;                     % @params: ub - upper bound vector of optimized parameters        
        weightTable;            % @params: weightTable - cost weighting vector
        nlIneqSelectionTable;   % @params: nlIneqSelectionTable - selection vector of nonlinear inequality constraints, taking value 0 or 1
        nlEqSelectionTable;     % @params: nlEqSelectionTable - selection vector of nonlinear equality constraints, taking value 0 or 1        
        optProbObj;             % @params: optProbObj - optimization problem object        
    end
    
    %% Ordinary Properties
    properties
        paramsEvaluated;
        sharedInformation;        
        costTable;
        nlIneqConstraintTable;
        nlEqConstraintTable;
        optimTool;        
        solverOption;
    end
      
    %% Abstract Methods
    methods(Abstract)         
        computeSharedInformation(obj, params)    
        % Compute shared information
        % @params: obj - object structure
        % @params: params - optimized parameters
        % @output: sharedInformation - shared information        
    end
    
    %% Ordinary Methods
    methods        
        %% Initializing Objects in Constructor        
        function obj = trajOptTool(params0, Aineq, bineq, Aeq, beq, lb, ub,...
                                 weightTable, nlIneqSelectionTable,...
                                 nlEqSelectionTable, optProbObj)
            
            obj.params0                 = params0;              
            obj.Aineq                   = Aineq;
            obj.bineq                   = bineq;
            obj.Aeq                     = Aeq;
            obj.beq                     = beq;
            obj.lb                      = lb;
            obj.ub                      = ub;
            obj.weightTable             = weightTable;
            obj.nlIneqSelectionTable    = nlIneqSelectionTable;
            obj.nlEqSelectionTable      = nlEqSelectionTable;            
            obj.optProbObj              = optProbObj;            
            obj.solverOption = optimoptions(@fmincon,...
                                            'Algorithm', 'sqp',...
                                            'FinDiffType', 'central',...
                                            'Display', 'iter-detailed',...
                                            'TolX', 1e-12,...
                                            'TolFun', 1e-12,...
                                            'TolCon', 1e-12);            
            
        end
        
        %% computeTotalCost
        % This function computes the total cost value        
        function costValue = computeTotalCost(obj, params)             
            % Check if computation is necessary  
            if ~isequal(params, obj.paramsEvaluated)                   
                obj.computeSharedInformation(params);
                obj.paramsEvaluated = params;
            end                        
            costValue    = 0;            
            if (size(obj.weightTable, 1) == size(obj.costTable, 2))
                for i1 = 1:size(obj.weightTable, 1)
                    if (obj.weightTable(i1) ~= 0)                                            
                        costValue = costValue + obj.weightTable(i1)*feval(obj.costTable{i1});                        
                    end
                end                
            else
                error('Dimensions of cost weighting vector and  number of cost functions must be equal');
            end            
        end
        
        %% computeNLConstraints
        % This function defines nonlinear inequality and equality constraints        
        function [nlIneq, nlEq] = computeNLConstraints(obj, params)
            % Check if computation is necessary  
            if ~isequal(params, obj.paramsEvaluated)                   
                obj.computeSharedInformation(params);
                obj.paramsEvaluated = params;
            end
            
            % Nonlinear inequality constraints
            nlIneq = [];
            if (size(obj.nlIneqSelectionTable, 1) == size(obj.nlIneqConstraintTable, 2))
                for i1 = 1:size(obj.nlIneqSelectionTable, 1)
                    if (obj.nlIneqSelectionTable(i1) == 1)                                        
                        nlIneq = [nlIneq;obj.nlIneqSelectionTable(i1)*feval(obj.nlIneqConstraintTable{i1})];                        
                    end
                end
            else
                error('Dimensions of nonlineq selection vector and  number of nonlineq functions must be equal');                
            end            
            % Nonlinear equality constraints
            nlEq = [];
            if (size(obj.nlEqSelectionTable, 1) == size(obj.nlEqConstraintTable, 2))
                for i1 = 1:size(obj.nlEqSelectionTable, 1)
                    if (obj.nlEqSelectionTable(i1) == 1)                                        
                        nlEq = [nlEq;obj.nlEqSelectionTable(i1)*feval(obj.nlEqConstraintTable{i1})];                        
                    end
                end
            else
                error('Dimensions of nonleq selection vector and  number of nonleq functions must be equal');                
            end
        end
        
        %% computeIpoptNLConstraints
        % This function defines nonlinear inequality and equality constraints        
        function [nlIpoptConstraints, nlIneqLength, nlEqLength] = computeIpoptNLConstraints(obj, params)
            % Check if computation is necessary  
            if ~isequal(params, obj.paramsEvaluated)                   
                obj.computeSharedInformation(params);
                obj.paramsEvaluated = params;
            end
            
            % Nonlinear inequality constraints
            nlIneq = [];
            if (size(obj.nlIneqSelectionTable, 1) == size(obj.nlIneqConstraintTable, 2))
                for i1 = 1:size(obj.nlIneqSelectionTable, 1)
                    if (obj.nlIneqSelectionTable(i1) == 1)                                        
                        nlIneq = [nlIneq;obj.nlIneqSelectionTable(i1)*feval(obj.nlIneqConstraintTable{i1})];                        
                    end
                end
            else
                error('Dimensions of nonlineq selection vector and  number of nonlineq functions must be equal');                
            end            
            % Nonlinear equality constraints
            nlEq = [];
            if (size(obj.nlEqSelectionTable, 1) == size(obj.nlEqConstraintTable, 2))
                for i1 = 1:size(obj.nlEqSelectionTable, 1)
                    if (obj.nlEqSelectionTable(i1) == 1)                                        
                        nlEq = [nlEq;obj.nlEqSelectionTable(i1)*feval(obj.nlEqConstraintTable{i1})];                        
                    end
                end
            else
                error('Dimensions of nonleq selection vector and  number of nonleq functions must be equal');                
            end
            nlIneqLength = length(nlIneq);
            nlEqLength = length(nlEq);
            nlIpoptConstraints = [nlIneq; nlEq];
        end
        
        
        %% solveParametricOptimization
        % This function is used to solve an optimization problem
        function [paramsOpt, costOpt, exitflag, output] = solveParametricOptimization(obj, optimTool)            
            %% Solving optimization problem
            if strcmp(optimTool.solver, 'fmincon')
                if strcmp(optimTool.searchMethod, '')
                    [paramsOpt, costOpt, exitflag, output] = fmincon(@(params)computeTotalCost(obj, params),...
                                                                     obj.params0, obj.Aineq, obj.bineq, obj.Aeq, obj.beq,...
                                                                     obj.lb, obj.ub, @(params)computeNLConstraints(obj, params), obj.solverOption);
                end
                if strcmp(optimTool.searchMethod, 'GlobalSearch')
                    optimizationProblem = createOptimProblem('fmincon','objective',@(params)computeTotalCost(obj, params),...
                                                             'x0', obj.params0, 'Aineq', obj.Aineq, 'bineq', obj.bineq, 'Aeq', obj.Aeq, 'beq', obj.beq,...
                                                             'lb', obj.lb, 'ub', obj.ub, 'nonlcon', @(params)computeNLConstraints(obj, params), 'options', obj.solverOption);
                    [paramsOpt, costOpt, exitflag, output] = run(GlobalSearch, optimizationProblem); % GlobalSearch
                end
                if strcmp(optimTool.searchMethod, 'MultiStart')
                    optimizationProblem = createOptimProblem('fmincon','objective',@(params)computeTotalCost(obj, params),...
                                                             'x0', obj.params0, 'Aineq', obj.Aineq, 'bineq', obj.bineq, 'Aeq', obj.Aeq, 'beq', obj.beq,...
                                                             'lb', obj.lb, 'ub', obj.ub, 'nonlcon', @(params)computeNLConstraints(obj, params), 'options', obj.solverOption);
                    ms = MultiStart;
                    % ms = MultiStart('PlotFcns', @gsplotbestf);
                    ms.StartPointsToRun = 'bounds';                    
                    [paramsOpt, costOpt, exitflag, output] = run(ms, optimizationProblem, optimTool.numTrialPoints); %  MultiStart                    
                end
            end
            if strcmp(optimTool.solver, 'ipopt')
                [~, nlIneqLength, nlEqLength] = computeIpoptNLConstraints(obj, zeros(size(obj.params0)));
                cl = [-Inf*ones(nlIneqLength, 1); zeros(nlEqLength, 1)];
                cu = zeros(nlIneqLength + nlEqLength, 1);                
                if optimTool.ipoptSolver
                    opts = optiset('solver', 'ipopt', 'display', 'iter');
                else
                    opts = optiset('solver', 'matlab', 'display', 'iter');
                end                
                prob = opti('fun', @(params)computeTotalCost(obj, params), 'nl', @(params)computeIpoptNLConstraints(obj, params), cl, cu, 'bounds', obj.lb, obj.ub, 'x0', obj.params0, 'options', opts);
                if optimTool.multiSolve
                    [paramsOpt, costOpt, exitflag, output] = multisolve(prob);
                else                    
                    [paramsOpt, costOpt, exitflag, output] = solve(prob);
                end                
            end
            if strcmp(optimTool.solver, 'nlopt')
                [~, nlIneqLength, nlEqLength] = computeIpoptNLConstraints(obj, zeros(size(obj.params0)));
                cl = [-Inf*ones(nlIneqLength, 1); zeros(nlEqLength, 1)];
                cu = zeros(nlIneqLength + nlEqLength, 1);                
                opts = optiset('solver', 'nlopt', 'solverOpts', nloptset('algorithm', 'AUGLAG'), 'display', 'iter');                
                prob = opti('fun', @(params)computeTotalCost(obj, params), 'nl', @(params)computeIpoptNLConstraints(obj, params), cl, cu, 'bounds', obj.lb, obj.ub, 'x0', obj.params0, 'options', opts);
                if optimTool.multiSolve
                    [paramsOpt, costOpt, exitflag, output] = multisolve(prob);
                else
                    [paramsOpt, costOpt, exitflag, output] = solve(prob);
                end                
            end
        end
    end
end
