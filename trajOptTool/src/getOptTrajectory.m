function optSysDynamics = getOptTrajectory(optProbObj)
%% Initialization
optimTool = optProbObj.optimTool;

%% Definition of linear constraints
% 1. Linear Inequality Constraints: Aineq*params <= bineq
Aineq = [];
bineq = [];
% 2. Linear Equality Constraints: Aeq*params = beq
Aeq = [];
beq = [];
% 3. Selection vector for costs and constraints
weightTable_tmp = 1;             
normalizedCost = 'YES';
%
if isequal(normalizedCost, 'YES')
    weightTable = zeros(size(weightTable_tmp, 1), 1);
    for i1 = 1:size(weightTable_tmp)
        weightTable(i1, 1) = weightTable_tmp(i1)/sum(weightTable_tmp);
    end
end
if isequal(normalizedCost, 'NO')
    weightTable = weightTable_tmp;
end
%
nlIneqSelectionTable  = 1;        % chosen nonlinear inequality constraints
nlEqSelectionTable    = 1;        % chosen nonlinear equality constraints            

%% Options for optimization algorithm(if desired - to change solver and its option)
solverOption = optimoptions(@fmincon, 'Algorithm', 'sqp', 'FinDiffType', 'central',...
                            'Display', 'iter-detailed', 'TolX', 1e-6,...
                            'TolFun', 1e-6, 'TolCon', 1e-6, 'MaxFunEvals', 1e+4);
%-------------------------------------------------------------------------%
% Remark: Available algorithms can be chosen
% 'interior-point'(default),'sqp','active-set','trust-region-reflective'
%-------------------------------------------------------------------------%

%% Lower and Upper bounds
sysDynamicsLB = optProbObj.optParamsBounds.lb;
lb = sysDynamics2optParams(sysDynamicsLB, optProbObj);
sysDynamicsUB = optProbObj.optParamsBounds.ub;
ub = sysDynamics2optParams(sysDynamicsUB, optProbObj);

%% Initial guess
sysDynamicsIC = optProbObj.optParamsIC.guess;
params0 = sysDynamics2optParams(sysDynamicsIC, optProbObj);

%% Optimization
trajectoryOpt = TrajectoryOpt(params0, Aineq, bineq, Aeq, beq, lb, ub,...
                              weightTable, nlIneqSelectionTable, nlEqSelectionTable,...
                              optProbObj);

trajectoryOpt.solverOption = solverOption;
trajectoryOpt.optimTool = optimTool;
optParams = trajectoryOpt.solveParametricOptimization(optimTool);

%% Results
optSysDynamics = optParams2sysDynamics(optParams, optProbObj);

end
%=========================================================================%
%============================ END OF PROGRAM =============================%
%=========================================================================%