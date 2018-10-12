%*************************************************************************%
% Project: ROMOCOTOOL
% Name: mainCartPoleModel.m
% Type: matlab script
% Version: 1.0
% Description: This script is a CartPoleModel test sample 
% Author: Quoc-Viet DANG
%*************************************************************************%
restoredefaultpath; warning off all; clearvars; close all; clc;

%% Add paths
roModCoToolRootPath = fileparts(fileparts(fileparts(fileparts([mfilename('fullpath'),'.m']))));
trajOptToolSrcPath = fullfile(roModCoToolRootPath, 'trajOptTool', 'src');
addpath(trajOptToolSrcPath);

%% System parameters
sysParams = getCartPoleModelSysParams();
nStates = sysParams.nStates;
nControls = sysParams.nControls;
nDirColGridPts = sysParams.nDirColGridPts;
d = sysParams.d;
dmax = sysParams.dmax;
umax = sysParams.umax;

%% Desired initial and final state
nJoints = 0.5*nStates;
desIniPos = zeros(nJoints, 1);
desIniVel = zeros(nJoints, 1);
desFinPos = [d 0;0 pi]*ones(nJoints, 1);
desFinVel = zeros(nJoints, 1);
desIniCon = zeros(nJoints, 1);
desFinCon = zeros(nJoints, 1);
%
desIniState = [desIniPos; desIniVel];
desFinState = [desFinPos; desFinVel];

%% Lower and Upper bounds
% Times
lb.t0 = 0;
ub.t0 = 0;
lb.tf = 0;
ub.tf = Inf;
% Positions
lb.positions = [-dmax 0;0 -pi]*ones(nJoints, nDirColGridPts);
ub.positions = [+dmax 0;0 +pi]*ones(nJoints, nDirColGridPts);
% Velocities
lb.velocities = -3*pi*ones(nJoints, nDirColGridPts);
ub.velocities = +3*pi*ones(nJoints, nDirColGridPts);
% Controls
lb.controls = -umax*ones(nControls, nDirColGridPts);
ub.controls = +umax*ones(nControls, nDirColGridPts);
%
optParamsBounds = getOptParamsBounds(sysParams, lb, ub);

%% Initial condition guess
% Times
guess.t0 = 0;
guess.tf = 2;
% Positions
guess.positions = [0.95*d 0;0 0.95*pi]*ones(nJoints, 1)*linspace(guess.t0, guess.tf, nDirColGridPts)/guess.tf;
% Velocities
guess.velocities = ones(nJoints, 1)*linspace(0, 1, nDirColGridPts);
% Controls
guess.controls = ones(nControls, 1)*ones(1, nDirColGridPts);
%
optParamsIC = getOptParamsIC(sysParams, guess);

%% Used optimization solver
optimTool.solver = 'fmincon';
optimTool.ipoptSolver = true;
optimTool.multiSolve = false;
optimTool.searchMethod = '';
optimTool.numTrialPoints = 10;

%% trajOpt2roModSysParams
trajOpt2roModSysParams = [sysParams.m1; sysParams.m2; sysParams.l; sysParams.g];

%% Optimization problem object
optProbObj.trajOpt2roModSysParams = trajOpt2roModSysParams;
optProbObj.sysParams = sysParams;
optProbObj.desIniState = desIniState;
optProbObj.desFinState = desFinState;
optProbObj.desIniCon = desIniCon;
optProbObj.desFinCon = desFinCon;
optProbObj.optParamsIC = optParamsIC;
optProbObj.optParamsBounds = optParamsBounds;
optProbObj.optimTool = optimTool;

%% Optimization
startOptimizationTimer = datetime;
optSysDynamics = getOptTrajectory(optProbObj);
finishedOptimizationTimer = datetime;
disp('******************************************************************');
disp('Finished trajectory generation!');
disp('Start time:'); disp(startOptimizationTimer);
disp('Finished time:'); disp(finishedOptimizationTimer);
disp('******************************************************************');

%% Interpolation
interpSysDynamics = getSysDynamicsInterpolation(optSysDynamics, optProbObj);
time = interpSysDynamics.time;
positions = interpSysDynamics.positions;
velocities = interpSysDynamics.velocities;
controls = interpSysDynamics.controls;

%% Plot
% Positions
figure;
for i1 = 1:nJoints
    subplot(nJoints, 1, i1); hold on; grid on;
    plot(time, positions(i1, :), 'b-');
    scatter(time(1), positions(i1, 1), 'ro');
    scatter(time(end), positions(i1, end), 'rx');
    xlabel('Time(s)');
    ylabel('Position');
end
suptitle('Positions');

% Velocities
figure;
for i1 = 1:nJoints
    subplot(nJoints, 1, i1); hold on; grid on;
    plot(time, velocities(i1, :), 'b-');
    scatter(time(1), velocities(i1, 1), 'ro');
    scatter(time(end), velocities(i1, end), 'rx');
    xlabel('Time(s)');
    ylabel('Velocities');
end
suptitle('Velocities');
% Controls
figure;
for i1 = 1:nControls
    subplot(nControls, 1, i1); hold on; grid on;
    plot(time, controls(i1, :), 'b-');
    scatter(time(1), controls(i1, 1), 'ro');
    scatter(time(end), controls(i1, end), 'rx');
    xlabel('Time(s)');
    ylabel('Controls');
end
suptitle('Controls');

%*************************************************************************%
return;
%=========================================================================%
%============================ END OF PROGRAM =============================%
%=========================================================================%