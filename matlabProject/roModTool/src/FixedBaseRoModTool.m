%*************************************************************************%
%=========================================================================%
% Project: ROMOCOTOOL
% Name: FixedBaseRoModTool.m
% Type: matlab object
% Version: 1.0
% Description: This object class provides methods for the robot modelling
% with a fixed base
% Author: Quoc-Viet DANG
%=========================================================================%
classdef FixedBaseRoModTool < RoModTool
    properties
        
    end    
        
    methods
        %% Constructor
        function obj = FixedBaseRoModTool(robot)
            
            obj = obj@RoModTool(robot);
            obj.robot = robot;
        end
        
        %*****************************************************************%
        %% getEulerLagrangeRoModTool
        function getEulerLagrangeRoModTool(obj, matFuncRootPath, simplifiedCode, sparseMatrix)
            if nargin <= 1
                error('Input argument error: matFuncRootPath(char), simplifiedCode(boolean), sparseMatrix(boolean) should be defined!');
            end
            if nargin <= 2
                error('Input argument error: simplifiedCode(boolean), sparseMatrix(boolean) should be defined!');
            end
            if nargin <= 3
                error('Input argument error: sparseMatrix(boolean) should be defined!');
            end            
            getRobotStructure(obj);
            getForwardKinematics(obj, simplifiedCode);
            getSymPhysicalParams(obj);
            getSymStateVariable(obj);
            getRobotEnergy(obj, simplifiedCode);
            getSymDynamicsVariable(obj);
            getSystemDynamics(obj, simplifiedCode);
            getMatlabFunction(obj, matFuncRootPath, sparseMatrix);
            
            %% getRobotStructure
            function getRobotStructure(obj)
                %% Base
                obj.robot.base.name = 'base';
                obj.robot.base.origin.xyz = sym(zeros(3, 1));
                obj.robot.base.origin.rpy = sym(zeros(3, 1));
                obj.robot.base.origin.dxyz = sym(zeros(3, 1));        
                obj.robot.base.origin.drpy = sym(zeros(3, 1));
                obj.robot.base.mass = sym(zeros(1));
                obj.robot.base.inertia = sym(zeros(3));

                %% Kinematic chain                                    
                nKinematicChains = size(obj.robot.kinematicChains, 2);    
                for i1 = nKinematicChains:-1:1
                    nJoints = size(obj.robot.kinematicChains(i1).joint, 2);
                    for i2 = nJoints:-1:1
                        obj.robot.kinematicChains(i1).joint(i2).name = ['joint', num2str(i1), num2str(i2)];
                        obj.robot.kinematicChains(i1).joint(i2).origin.xyz = str2sym([['j', num2str(i1), num2str(i2), 'xp']; ['j', num2str(i1), num2str(i2), 'yp']; ['j', num2str(i1), num2str(i2), 'zp']]);
                        obj.robot.kinematicChains(i1).joint(i2).origin.rpy = str2sym([['j', num2str(i1), num2str(i2), 'ro']; ['j', num2str(i1), num2str(i2), 'po']; ['j', num2str(i1), num2str(i2), 'yo']]);
                        obj.robot.kinematicChains(i1).joint(i2).parentlink = ['link', mat2str([i1, i2-1])];
                        obj.robot.kinematicChains(i1).joint(i2).childlink = ['link', num2str(i1), num2str(i2)];
                        obj.robot.kinematicChains(i1).joint(i2).symPhysicalParams = [obj.robot.kinematicChains(i1).joint(i2).origin.xyz;...
                                                                                     obj.robot.kinematicChains(i1).joint(i2).origin.rpy];
                        %
                        obj.robot.kinematicChains(i1).link(i2).name = ['link', num2str(i1), num2str(i2)];
                        obj.robot.kinematicChains(i1).link(i2).origin.xyz = str2sym([['l', num2str(i1), num2str(i2), 'xp']; ['l', num2str(i1), num2str(i2), 'yp']; ['l', num2str(i1), num2str(i2), 'zp']]);
                        obj.robot.kinematicChains(i1).link(i2).origin.rpy = str2sym([['l', num2str(i1), num2str(i2), 'ro']; ['l', num2str(i1), num2str(i2), 'po']; ['l', num2str(i1), num2str(i2), 'yo']]);
                        obj.robot.kinematicChains(i1).link(i2).mass = str2sym(['m', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(1, 1) = str2sym(['ixx', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(1, 2) = str2sym(['ixy', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(1, 3) = str2sym(['ixz', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(2, 1) = str2sym(['ixy', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(2, 2) = str2sym(['iyy', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(2, 3) = str2sym(['iyz', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(3, 1) = str2sym(['ixz', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(3, 2) = str2sym(['iyz', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).inertia(3, 3) = str2sym(['izz', num2str(i1), num2str(i2)]);
                        obj.robot.kinematicChains(i1).link(i2).symPhysicalParams = [obj.robot.kinematicChains(i1).link(i2).origin.xyz;...
                                                                                    obj.robot.kinematicChains(i1).link(i2).origin.rpy;...
                                                                                    obj.robot.kinematicChains(i1).link(i2).mass;...
                                                                                    obj.robot.kinematicChains(i1).link(i2).inertia(1, 1);...
                                                                                    obj.robot.kinematicChains(i1).link(i2).inertia(1, 2);...
                                                                                    obj.robot.kinematicChains(i1).link(i2).inertia(1, 3);...
                                                                                    obj.robot.kinematicChains(i1).link(i2).inertia(2, 2);...
                                                                                    obj.robot.kinematicChains(i1).link(i2).inertia(2, 3);...
                                                                                    obj.robot.kinematicChains(i1).link(i2).inertia(3, 3)];
                    end
                end
            end

            %% getForwardKinematics
            function getForwardKinematics(obj, simplifiedCode)
                %% Base
                basePos = obj.robot.base.origin.xyz;
                baseOri = obj.robot.base.origin.rpy;
                baseRot = Rpy2Mat(baseOri);

                %% Kinematic chain
                nKinematicChains = size(obj.robot.kinematicChains, 2);
                nChainJoints = zeros(nKinematicChains, 1);
                for i1 = 1:nKinematicChains
                    nChainJoints(i1) = size(obj.robot.kinematicChains(i1).joint, 2);
                end
                nChainJoints = [0; nChainJoints];    
                for i1 = nKinematicChains:-1:1
                    nJoints = size(obj.robot.kinematicChains(i1).joint, 2);
                    % Symbolic variable definition
                    jointRot = sym(zeros(3, 3, nJoints));
                    jointPos = sym(zeros(3, 1, nJoints));
                    jointVel = sym(zeros(3, 1, nJoints));
                    linkRot = sym(zeros(3, 3, nJoints));
                    linkPos = sym(zeros(3, 1, nJoints));
                    linkVel = sym(zeros(3, 1, nJoints));
                    jointVar = sym(['q', num2str(i1)], [nJoints, 1]);
                    dJointVar = sym(['dq', num2str(i1)], [nJoints, 1]);                    
                    % Extended joint variable
                    exJointVar = jointVar;
                    dExJointVar = dJointVar;
                    jointPosJaco = sym(zeros(3, nJoints, nJoints));
                    linkPosJaco = sym(zeros(3, nJoints, nJoints));
                    % Joint1
                    jointRot(:, :, 1) = baseRot*Rpy2Mat(obj.robot.kinematicChains(i1).joint(1).origin.rpy);
                    jointPos(:, :, 1) = (basePos + baseRot*obj.robot.kinematicChains(i1).joint(1).origin.xyz);
                    jointPosJaco(:, :, 1) = jacobian(jointPos(:, :, 1), exJointVar);
                    jointVel(:, :, 1) = jointPosJaco(:, :, 1)*dExJointVar;
                    % Link1
                    link1Rq = strcmp(obj.robot.kinematicChains(i1).joint(1).type, 'revolute')*Rpy2Mat(obj.robot.kinematicChains(i1).joint(1).axis*jointVar(1)) + strcmp(obj.robot.kinematicChains(i1).joint(1).type, 'prismatic')*eye(3);
                    linkRot(:, :, 1) = jointRot(:, :, 1)*Rpy2Mat(obj.robot.kinematicChains(i1).link(1).origin.rpy)*link1Rq;
                    linkPos(:, :, 1) = jointPos(:, :, 1) + jointRot(:, :, 1)*link1Rq*obj.robot.kinematicChains(i1).link(1).origin.xyz;
                    linkPosJaco(:, :, 1) = jacobian(linkPos(:, :, 1), exJointVar);
                    linkVel(:, :, 1) = linkPosJaco(:, :, 1)*dExJointVar;
                    % Other Joints and Links
                    for i2 = 2:nJoints
                        % Joints
                        jointRrpy = Rpy2Mat(obj.robot.kinematicChains(i1).joint(i2-1).origin.rpy);
                        jointRq = strcmp(obj.robot.kinematicChains(i1).joint(i2-1).type, 'revolute')*Rpy2Mat(obj.robot.kinematicChains(i1).joint(i2-1).axis*jointVar(i2-1)) + strcmp(obj.robot.kinematicChains(i1).joint(i2-1).type, 'prismatic')*eye(3);
                        jointRot(:, :, i2) = jointRot(:, :, i2-1)*jointRrpy*jointRq;
                        jointPxyz = obj.robot.kinematicChains(i1).joint(i2).origin.xyz;
                        jointPq = strcmp(obj.robot.kinematicChains(i1).joint(i2).type, 'prismatic')*(obj.robot.kinematicChains(i1).joint(i2).axis*jointVar(i2-1));            
                        jointPos(:, :, i2) = jointPos(:, :, i2-1) + jointRot(:, :, i2-1)*jointRq*(jointPxyz + jointPq);
                        jointPosJaco(:, :, i2) = jacobian(jointPos(:, :, i2), exJointVar);
                        jointVel(:, :, i2) = jointPosJaco(:, :, i2)*dExJointVar;
                        % Links
                        linkRq = strcmp(obj.robot.kinematicChains(i1).joint(i2).type, 'revolute')*Rpy2Mat(obj.robot.kinematicChains(i1).joint(i2).axis*jointVar(i2)) + strcmp(obj.robot.kinematicChains(i1).joint(i2).type, 'prismatic')*eye(3);
                        linkRot(:, :, i2) = jointRot(:, :, i2)*Rpy2Mat(obj.robot.kinematicChains(i1).link(i2).origin.rpy)*linkRq;            
                        linkPos(:, :, i2) = jointPos(:, :, i2) + jointRot(:, :, i2)*Rpy2Mat(obj.robot.kinematicChains(i1).link(i2).origin.rpy)*linkRq*obj.robot.kinematicChains(i1).link(i2).origin.xyz;
                        linkPosJaco(:, :, i2) = jacobian(linkPos(:, :, i2), exJointVar);            
                        linkVel(:, :, i2) = linkPosJaco(:, :, i2)*dExJointVar;
                    end
                    tmpJointPosJaco = sym(zeros(size(jointPosJaco)));
                    tmpLinkPosJaco = sym(zeros(size(linkPosJaco)));        
                    tmpJointPosJaco(:, sum(nChainJoints(1:i1, 1))+1:sum(nChainJoints(1:(i1+1), 1)), :) = jointPosJaco;        
                    tmpLinkPosJaco(:, sum(nChainJoints(1:i1, 1))+1:sum(nChainJoints(1:(i1+1), 1)), :) = linkPosJaco;        
                    % Forward Kinematics
                    if simplifiedCode
                        obj.robot.forwardKinematics.chain(i1).jointRot = simplify(jointRot);
                        obj.robot.forwardKinematics.chain(i1).jointPos = simplify(jointPos);
                        obj.robot.forwardKinematics.chain(i1).jointPosJaco = simplify(tmpJointPosJaco);
                        obj.robot.forwardKinematics.chain(i1).jointVel = simplify(jointVel);
                        obj.robot.forwardKinematics.chain(i1).linkRot = simplify(linkRot);
                        obj.robot.forwardKinematics.chain(i1).linkPos = simplify(linkPos);
                        obj.robot.forwardKinematics.chain(i1).linkPosJaco = simplify(tmpLinkPosJaco);
                        obj.robot.forwardKinematics.chain(i1).linkVel = simplify(linkVel);                        
                    else
                        obj.robot.forwardKinematics.chain(i1).jointRot = jointRot;
                        obj.robot.forwardKinematics.chain(i1).jointPos = jointPos;
                        obj.robot.forwardKinematics.chain(i1).jointPosJaco = tmpJointPosJaco;
                        obj.robot.forwardKinematics.chain(i1).jointVel = jointVel;
                        obj.robot.forwardKinematics.chain(i1).linkRot = linkRot;
                        obj.robot.forwardKinematics.chain(i1).linkPos = linkPos;
                        obj.robot.forwardKinematics.chain(i1).linkPosJaco = tmpLinkPosJaco;
                        obj.robot.forwardKinematics.chain(i1).linkVel = linkVel;
                        
                    end
                    obj.robot.forwardKinematics.chain(i1).symJointVar = jointVar;
                    obj.robot.forwardKinematics.chain(i1).symdJointVar = dJointVar;                    
                end
            end

            %% getSymPhysicalParams
            function getSymPhysicalParams(obj)
                %% Gravity acceleration
                symGravityAccel = sym('g');

                %% Symbolic physical parameters
                nKinematicChains = size(obj.robot.kinematicChains, 2);   
                symJointPhysicalParams = [];
                symLinkPhysicalParams = [];
                for i1 = 1:nKinematicChains
                    nJoints = size(obj.robot.kinematicChains(i1).joint, 2);
                    for i2 = 1:nJoints
                        symJointPhysicalParams = [symJointPhysicalParams; obj.robot.kinematicChains(i1).joint(i2).symPhysicalParams]; %#ok<AGROW>
                        symLinkPhysicalParams = [symLinkPhysicalParams; obj.robot.kinematicChains(i1).link(i2).symPhysicalParams]; %#ok<AGROW>
                    end
                end    
                obj.robot.symPhysicalParams = [symGravityAccel; symJointPhysicalParams; symLinkPhysicalParams];
            end

            %% getSymStateVariable
            function getSymStateVariable(obj)
                %% Symbolic state variable
                nKinematicChains = size(obj.robot.kinematicChains, 2);
                symJointVars = [];
                symdJointVars = [];
                for i1 = 1:nKinematicChains
                    symJointVars = [symJointVars; obj.robot.forwardKinematics.chain(i1).symJointVar]; %#ok<AGROW>
                    symdJointVars = [symdJointVars; obj.robot.forwardKinematics.chain(i1).symdJointVar]; %#ok<AGROW>
                    
                end            
                obj.robot.symStateVariable = [symJointVars; symdJointVars];                
            end            

            %% getRobotEnergy
            function getRobotEnergy(obj, simplifiedCode)
                %% Gravity acceleration
                symGravityAccel = obj.robot.symPhysicalParams(1);

                %% Base
                kinematicEnergy = sym(zeros(1));
                potentialEnergy = sym(zeros(1));

                %% Kinematic chain    
                nKinematicChains = size(obj.robot.kinematicChains, 2);    
                for i1 = 1:nKinematicChains        
                    nJoints = size(obj.robot.kinematicChains(i1).joint, 2);        
                    for i2 = 1:nJoints
                        linkIc = obj.robot.kinematicChains(i1).link(i2).inertia;
                        linkR = obj.robot.kinematicChains(i1).link(i2).origin.xyz;
                        linkM = obj.robot.kinematicChains(i1).link(i2).mass;            
                        linkV = obj.robot.forwardKinematics.chain(i1).linkVel(:, :, i2);
                        linkId = linkIc + linkM*(linkR'*linkR*eye(3) - linkR*linkR');
                        linkW = obj.robot.forwardKinematics.chain(i1).symJointVar(i2)*obj.robot.kinematicChains(i1).joint(i2).axis;
                        kinematicEnergy = kinematicEnergy + 0.5*(linkM*(linkV'*linkV) + linkW'*linkId*linkW);
                        potentialEnergy = potentialEnergy + linkM*symGravityAccel*obj.robot.forwardKinematics.chain(i1).linkPos(3, :, i2);
                    end           
                end
                if simplifiedCode
                    obj.robot.Lagrangian.kinematicEnergy = simplify(kinematicEnergy);
                    obj.robot.Lagrangian.potentialEnergy = simplify(potentialEnergy);
                else
                    obj.robot.Lagrangian.kinematicEnergy = kinematicEnergy;
                    obj.robot.Lagrangian.potentialEnergy = potentialEnergy;
                end
            end

            %% getSystemDynamics
            function getSystemDynamics(obj, simplifiedCode)
                %% Kinematic and potential energy
                Ek = obj.robot.Lagrangian.kinematicEnergy;
                Ep = obj.robot.Lagrangian.potentialEnergy;

                %% System dynamics
                states = obj.robot.symStateVariable;
                nStates = length(states);
                nJoints = 0.5*nStates;
                positions = states(1:nJoints, 1);
                velocities = states((nJoints+1):end, 1);

                %% Dynamical matrices
                % Inertia matrix
                M = sym(zeros(nJoints, nJoints));
                for i1 = 1:nJoints
                    for j1 = 1:nJoints
                        % M(i1, j1) = simplify(diff(diff(Ek, velocities(j1, 1)), velocities(i1, 1)));
                        M(i1, j1) = diff(diff(Ek, velocities(j1, 1)), velocities(i1, 1));
                    end
                end
                % Coriolis matrix
                C1 = sym(zeros(nJoints, 1));
                C2 = sym(zeros(nJoints, 1));
                C3 = sym(zeros(nJoints, 1));
                C = sym(zeros(nJoints, nJoints));
                for i1 = 1:nJoints
                    for j1 = 1:nJoints        
                        C4 = 0;
                        for k = 1:nJoints            
                            C1(k, 1) = 0.5*diff(M(i1, j1), positions(k, 1))*velocities(k, 1);
                            C2(k, 1) = 0.5*diff(M(i1, k), positions(j1, 1))*velocities(k, 1);
                            C3(k, 1) = 0.5*diff(M(j1, k), positions(i1, 1))*velocities(k, 1);            
                            C4 = C4 + C1(k) + C2(k) - C3(k);
                        end
                        % C(i1, j1) = simplify(C4);
                        C(i1, j1) = C4;
                    end
                end
                % Gravity matrix
                G = sym(zeros(nJoints, 1));
                for i1 = 1:nJoints
                    % G(i1, 1) = simplify(diff(Ep, positions(i1)));
                    G(i1, 1) = diff(Ep, positions(i1));
                end            
                obj.robot.systemDynamics.nStates = nStates;
                if simplifiedCode
                    obj.robot.systemDynamics.M = simplify(M);
                    obj.robot.systemDynamics.C = simplify(C);
                    obj.robot.systemDynamics.G = simplify(G);
                else
                    obj.robot.systemDynamics.M = M;
                    obj.robot.systemDynamics.C = C;
                    obj.robot.systemDynamics.G = G;
                end
                
            end 
            
            %% getSymDynamicsVariable
            function getSymDynamicsVariable(obj)                
                nKinematicChains = size(obj.robot.kinematicChains, 2);
                symAccelVars = [];
                symControlVars = [];
                for i1 = 1:nKinematicChains
                    nJoints = size(obj.robot.kinematicChains(i1).joint, 2);
                    symAccelVars = [symAccelVars; sym(['d2q', num2str(i1)], [nJoints, 1])]; %#ok<AGROW>
                    symControlVars = [symControlVars; sym(['u', num2str(i1)], [nJoints, 1])]; %#ok<AGROW>
                end            
                obj.robot.symAccelerationVariable = symAccelVars;
                obj.robot.symControlVariable = symControlVars;
            end            

            %% getMatlabFunction
            function getMatlabFunction(obj, matFuncRootPath, sparseMatrix)
                %% matlabFunction
                nKinematicChains = size(obj.robot.kinematicChains, 2);
                symInputVars = {obj.robot.symPhysicalParams, obj.robot.symStateVariable};
                for i1 = 1:nKinematicChains
                    nJoints = size(obj.robot.kinematicChains(i1).joint, 2);
                    for i2 = 1:nJoints
                        % getJointPosition                    
                        jointPosMatFuncName = ['getJoint', num2str(i1), num2str(i2), 'Position'];
                        jointPosSymMatFuncPath = fullfile(matFuncRootPath, [jointPosMatFuncName, '.m']);                    
                        jointPosOutputVars = obj.robot.forwardKinematics.chain(i1).jointPos(:, :, i2);                    
                        jointPosHeader = {'======================================================================%';...
                              [' Description: This function computes the position of joint', num2str(i1), num2str(i2)];...
                              [' joint', num2str(i1), num2str(i2), 'Pos ', '= ', jointPosMatFuncName, '(physicalParams, stateVariable)'];...
                              ' @Inputs:';...                           
                              ' physicalParams: Physical parameters of the dynamical system - column vector';...
                              ' stateVariable: State variable of the dynamical system - column vector';...
                              ' @Output:';...
                              [' joint', num2str(i1), num2str(i2), 'Pos: ', 'Position of the joint', num2str(i1), num2str(i2), ' - column vector'];...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                        if sparseMatrix
                            matlabFunction(jointPosOutputVars, 'File', jointPosSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['joint', num2str(i1), num2str(i2), 'Pos']}, 'Comments', jointPosHeader, 'Sparse', true);
                        else
                            matlabFunction(jointPosOutputVars, 'File', jointPosSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['joint', num2str(i1), num2str(i2), 'Pos']}, 'Comments', jointPosHeader);
                        end
                        % getJointPositionJacobian
                        jointPosJacoMatFuncName = ['getJoint', num2str(i1), num2str(i2), 'PositionJacobian'];
                        jointPosJacoSymMatFuncPath = fullfile(matFuncRootPath, [jointPosJacoMatFuncName, '.m']);                    
                        jointPosJacoOutputVars = obj.robot.forwardKinematics.chain(i1).jointPosJaco(:, :, i2);
                        jointPosJacoHeader = {'======================================================================%';...
                              [' Description: This function computes the position jacobian of joint', num2str(i1), num2str(i2)];...
                              [' joint', num2str(i1), num2str(i2), 'PosJaco ', '= ', jointPosJacoMatFuncName, '(physicalParams, stateVariable)'];...
                              ' @Inputs:';...                          
                              ' physicalParams: Physical parameters of the dynamical system - column vector';...
                              ' stateVariable: State variable of the dynamical system - column vector';...
                              ' @Output:';...
                              [' joint', num2str(i1), num2str(i2), 'PosJaco: ', 'Position jacobian of the joint', num2str(i1), num2str(i2), ' - column vector'];...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };  
                        if sparseMatrix
                            matlabFunction(jointPosJacoOutputVars, 'File', jointPosJacoSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['joint', num2str(i1), num2str(i2), 'PosJaco']}, 'Comments', jointPosJacoHeader, 'Sparse', true);
                        else
                            matlabFunction(jointPosJacoOutputVars, 'File', jointPosJacoSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['joint', num2str(i1), num2str(i2), 'PosJaco']}, 'Comments', jointPosJacoHeader);
                        end
                        % getJointVelocity
                        jointVelMatFuncName = ['getJoint', num2str(i1), num2str(i2), 'Velocity'];
                        jointVelSymMatFuncPath = fullfile(matFuncRootPath, [jointVelMatFuncName, '.m']);                    
                        jointVelOutputVars = obj.robot.forwardKinematics.chain(i1).jointVel(:, :, i2);
                        jointVelHeader = {'======================================================================%';...
                              [' Description: This function computes the velocity of joint', num2str(i1), num2str(i2)];...
                              [' joint', num2str(i1), num2str(i2), 'Vel ', '= ', jointVelMatFuncName, '(physicalParams, stateVariable)'];...
                              ' @Inputs:';...                            
                              ' physicalParams: Physical parameters of the dynamical system - column vector';...
                              ' stateVariable: State variable of the dynamical system - column vector';...
                              ' @Output:';...
                              [' joint', num2str(i1), num2str(i2), 'Vel: ', 'Velocity of the joint', num2str(i1), num2str(i2), ' - column vector'];...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                        if sparseMatrix
                            matlabFunction(jointVelOutputVars, 'File', jointVelSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['joint', num2str(i1), num2str(i2), 'Vel']}, 'Comments', jointVelHeader, 'Sparse', true);
                        else
                            matlabFunction(jointVelOutputVars, 'File', jointVelSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['joint', num2str(i1), num2str(i2), 'Vel']}, 'Comments', jointVelHeader);
                        end
                        % getJointRotation
                        jointRotMatFuncName = ['getJoint', num2str(i1), num2str(i2), 'Rotation'];
                        jointRotSymMatFuncPath = fullfile(matFuncRootPath, [jointRotMatFuncName, '.m']);                    
                        jointRotOutputVars = obj.robot.forwardKinematics.chain(i1).jointRot(:, :, i2);
                        jointRotHeader = {'======================================================================%';...
                              [' Description: This function computes the rotation matrix of joint', num2str(i1), num2str(i2)];...
                              [' joint', num2str(i1), num2str(i2), 'Rot ', '= ', jointRotMatFuncName, '(physicalParams, stateVariable)'];...
                              ' @Inputs:';...                              
                              ' physicalParams: Physical parameters of the dynamical system - column vector';...
                              ' stateVariable: State variable of the dynamical system - column vector';...
                              ' @Output:';...
                              [' joint', num2str(i1), num2str(i2), 'Rot: ', 'Rotation matrix of the joint', num2str(i1), num2str(i2), ' - 3x3 matrix'];...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                        if sparseMatrix
                            matlabFunction(jointRotOutputVars, 'File', jointRotSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['joint', num2str(i1), num2str(i2), 'Rot']}, 'Comments', jointRotHeader, 'Sparse', true);
                        else
                            matlabFunction(jointRotOutputVars, 'File', jointRotSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['joint', num2str(i1), num2str(i2), 'Rot']}, 'Comments', jointRotHeader);
                        end
                        % getLinkPosition
                        linkPosMatFuncName = ['getLink', num2str(i1), num2str(i2), 'Position'];
                        linkPosSymMatFuncPath = fullfile(matFuncRootPath, [linkPosMatFuncName, '.m']);                    
                        linkPosOutputVars = obj.robot.forwardKinematics.chain(i1).linkPos(:, :, i2);
                        linkPosHeader = {'======================================================================%';...
                              [' Description: This function computes the position of the link', num2str(i1), num2str(i2)];...
                              [' link', num2str(i1), num2str(i2), 'Pos ', '= ', linkPosMatFuncName, '(physicalParams, stateVariable)'];...
                              ' @Inputs:';...                              
                              ' physicalParams: Physical parameters of the dynamical system - column vector';...
                              ' stateVariable: State variable of the dynamical system - column vector';...
                              ' @Output:';...
                              [' link', num2str(i1), num2str(i2), 'Pos: ', 'Position of the link', num2str(i1), num2str(i2), ' - column vector'];...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              }; 
                        if sparseMatrix
                            matlabFunction(linkPosOutputVars, 'File', linkPosSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['link', num2str(i1), num2str(i2), 'Pos']}, 'Comments', linkPosHeader, 'Sparse', true);
                        else
                            matlabFunction(linkPosOutputVars, 'File', linkPosSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['link', num2str(i1), num2str(i2), 'Pos']}, 'Comments', linkPosHeader);
                        end
                        % getLinkPositionJacobian
                        linkPosJacoMatFuncName = ['getLink', num2str(i1), num2str(i2), 'PositionJacobian'];
                        linkPosJacoSymMatFuncPath = fullfile(matFuncRootPath, [linkPosJacoMatFuncName, '.m']);                    
                        linkPosJacoOutputVars = obj.robot.forwardKinematics.chain(i1).linkPosJaco(:, :, i2);
                        linkPosJacoHeader = {'======================================================================%';...
                              [' Description: This function computes the position jacobian of the link', num2str(i1), num2str(i2)];...
                              [' link', num2str(i1), num2str(i2), 'Pos ', '= ', linkPosJacoMatFuncName, '(physicalParams, stateVariable)'];...
                              ' @Inputs:';...                                        
                              ' physicalParams: Physical parameters of the dynamical system - column vector';...
                              ' stateVariable: State variable of the dynamical system - column vector';...
                              ' @Output:';...
                              [' link', num2str(i1), num2str(i2), 'Pos: ', 'Position jacobian of the link', num2str(i1), num2str(i2), ' - column vector'];...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                        if sparseMatrix
                            matlabFunction(linkPosJacoOutputVars, 'File', linkPosJacoSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['link', num2str(i1), num2str(i2), 'PosJaco']}, 'Comments', linkPosJacoHeader, 'Sparse', true);
                        else
                            matlabFunction(linkPosJacoOutputVars, 'File', linkPosJacoSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['link', num2str(i1), num2str(i2), 'PosJaco']}, 'Comments', linkPosJacoHeader);
                        end
                        % getLinkVelocity
                        linkVelMatFuncName = ['getLink', num2str(i1), num2str(i2), 'Velocity'];
                        linkVelSymMatFuncPath = fullfile(matFuncRootPath, [linkVelMatFuncName, '.m']);                    
                        linkVelOutputVars = obj.robot.forwardKinematics.chain(i1).linkVel(:, :, i2);
                        linkVelHeader = {'======================================================================%';...
                              [' Description: This function computes the velocity of the link', num2str(i1), num2str(i2)];...
                              [' link', num2str(i1), num2str(i2), 'Vel ', '= ', linkVelMatFuncName, '(physicalParams, stateVariable)'];...
                              ' @Inputs:';...                                
                              ' physicalParams: Physical parameters of the dynamical system - column vector';...
                              ' stateVariable: State variable of the dynamical system - column vector';...
                              ' @Output:';...
                              [' link', num2str(i1), num2str(i2), 'Vel: ', 'Velocity of the link', num2str(i1), num2str(i2), ' - column vector'];...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                        if sparseMatrix
                            matlabFunction(linkVelOutputVars, 'File', linkVelSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['link', num2str(i1), num2str(i2), 'Vel']}, 'Comments', linkVelHeader, 'Sparse', true);
                        else
                            matlabFunction(linkVelOutputVars, 'File', linkVelSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['link', num2str(i1), num2str(i2), 'Vel']}, 'Comments', linkVelHeader);
                        end
                        % getLinkRotation
                        linkRotMatFuncName = ['getLink', num2str(i1), num2str(i2), 'Rotation'];
                        linkRotSymMatFuncPath = fullfile(matFuncRootPath, [linkRotMatFuncName, '.m']);                    
                        linkRotOutputVars = obj.robot.forwardKinematics.chain(i1).linkRot(:, :, i2);
                        linkRotHeader = {'======================================================================%';...
                              [' Description: This function computes the rotation matrix of the link', num2str(i1), num2str(i2)];...
                              [' link', num2str(i1), num2str(i2), 'Rot ', '= ', linkRotMatFuncName, '(physicalParams, stateVariable)'];...
                              ' @Inputs:';...                             
                              ' physicalParams: Physical parameters of the dynamical system - column vector';...
                              ' stateVariable: State variable of the dynamical system - column vector';...
                              ' @Output:';...
                              [' link', num2str(i1), num2str(i2), 'Rot: ', 'Rotation matrix of the link', num2str(i1), num2str(i2), ' - 3x3 matrix'];...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                        if sparseMatrix
                            matlabFunction(linkRotOutputVars, 'File', linkRotSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['link', num2str(i1), num2str(i2), 'Rot']}, 'Comments', linkRotHeader, 'Sparse', true);
                        else
                            matlabFunction(linkRotOutputVars, 'File', linkRotSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {['link', num2str(i1), num2str(i2), 'Rot']}, 'Comments', linkRotHeader);
                        end
                    end
                end
                % getInertiaMatrix
                inertiaMatrixMatFuncName = 'getInertiaMatrix';
                inertiaMatrixSymMatFuncPath = fullfile(matFuncRootPath, [inertiaMatrixMatFuncName, '.m']);                    
                inertiaMatrixOutputVars = obj.robot.systemDynamics.M ;
                inertiaMatrixHeader = {'======================================================================%';...
                      ' Description: This function computes the inertia matrix M of robot dynamics';...
                      [' inertiaMatrix ', '= ', inertiaMatrixMatFuncName, '(physicalParams, stateVariable)'];...
                      ' @Inputs:';...                         
                      ' physicalParams: Physical parameters of the dynamical system - column vector';...
                      ' stateVariable: State variable of the dynamical system - column vector';...
                      ' @Output:';...
                      [' inertiaMatrix: ', 'Inertia matrix of robot dynamics'];...
                      ' Version: 1.0';...
                      ' Author: Quoc-Viet Dang';...
                      '======================================================================%';...
                      };
                if sparseMatrix
                    matlabFunction(inertiaMatrixOutputVars, 'File', inertiaMatrixSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {'inertiaMatrix'}, 'Comments', inertiaMatrixHeader, 'Sparse', true);
                else
                    matlabFunction(inertiaMatrixOutputVars, 'File', inertiaMatrixSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {'inertiaMatrix'}, 'Comments', inertiaMatrixHeader);
                end
                % getCoriolisMatrix
                coriolisMatrixMatFuncName = 'getCoriolisMatrix';
                coriolisMatrixSymMatFuncPath = fullfile(matFuncRootPath, [coriolisMatrixMatFuncName, '.m']);                    
                coriolisMatrixOutputVars = obj.robot.systemDynamics.C ;
                coriolisMatrixHeader = {'======================================================================%';...
                      ' Description: This function computes the Coriolis matrix C of robot dynamics';...
                      [' coriolisMatrix ', '= ', coriolisMatrixMatFuncName, '(physicalParams, stateVariable)'];...
                      ' @Inputs:';...                            
                      ' physicalParams: Physical parameters of the dynamical system - column vector';...
                      ' stateVariable: State variable of the dynamical system - column vector';...
                      ' @Output:';...
                      [' coriolisMatrix: ', 'Coriolis matrix of robot dynamics'];...
                      ' Version: 1.0';...
                      ' Author: Quoc-Viet Dang';...
                      '======================================================================%';...
                      };
                if sparseMatrix
                    matlabFunction(coriolisMatrixOutputVars, 'File', coriolisMatrixSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {'coriolisMatrix'}, 'Comments', coriolisMatrixHeader, 'Sparse', true);
                else
                    matlabFunction(coriolisMatrixOutputVars, 'File', coriolisMatrixSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {'coriolisMatrix'}, 'Comments', coriolisMatrixHeader);
                end
                % getGravityTorque
                gravityTorqueMatFuncName = 'getGravityTorque';
                gravityTorqueSymMatFuncPath = fullfile(matFuncRootPath, [gravityTorqueMatFuncName, '.m']);                    
                gravityTorqueOutputVars = obj.robot.systemDynamics.G ;
                gravityTorqueHeader = {'======================================================================%';...
                      ' Description: This function computes the gravity torque G of robot dynamics';...
                      [' gravityTorque ', '= ', gravityTorqueMatFuncName, '(physicalParams, stateVariable)'];...
                      ' @Inputs:';...                    
                      ' physicalParams: Physical parameters of the dynamical system - column vector';...
                      ' stateVariable: State variable of the dynamical system - column vector';...
                      ' @Output:';...
                      [' gravityTorque: ', 'Gravity torque of robot dynamics'];...
                      ' Version: 1.0';...
                      ' Author: Quoc-Viet Dang';...
                      '======================================================================%';...
                      };
                if sparseMatrix
                    matlabFunction(gravityTorqueOutputVars, 'File', gravityTorqueSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {'gravityTorque'}, 'Comments', gravityTorqueHeader, 'Sparse', true);
                else
                    matlabFunction(gravityTorqueOutputVars, 'File', gravityTorqueSymMatFuncPath, 'Vars', symInputVars, 'Outputs', {'gravityTorque'}, 'Comments', gravityTorqueHeader);
                end                
                % State-space model                        
                nStates = obj.robot.systemDynamics.nStates;
                nJoints = 0.5*nStates;                
                velocities = obj.robot.symStateVariable((nJoints+1):end, 1);
                M = sym('M', [nJoints, nJoints]); C = sym('C', [nJoints, nJoints]); G = sym('G', [nJoints, 1]);
                dStates = [velocities; M\(obj.robot.symControlVariable - C*velocities - G)];
                stateSpaceModPath = fullfile(matFuncRootPath, 'getStateSpaceModel.m');
                stateSpaceModInputVars = {[M, C, G], obj.robot.symStateVariable, obj.robot.symControlVariable};
                stateSpaceModFuncHeader = {'======================================================================%';...
                              ' Description: This function gives the state-space model of robot';...
                              ' dStates = getStateSpaceModel([M, C, G], states, controls)';...
                              ' @Inputs:';...                              
                              ' M: Inertia matrix of the dynamical system';...
                              ' C: Coriolis matrix of the dynamical system';...
                              ' G: Gravity matrix of the dynamical system';...
                              ' states: State variable of the dynamical system - column vector';...
                              ' controls: Control inputs of the dynamical system - column vector';...
                              ' @Output:';...
                              ' dStates: Time derivative of state variable of the dynamical system';...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                if sparseMatrix
                    matlabFunction(dStates, 'File', stateSpaceModPath, 'Vars', stateSpaceModInputVars, 'Outputs', {'dStates'}, 'Comments', stateSpaceModFuncHeader, 'Sparse', true);
                else
                    matlabFunction(dStates, 'File', stateSpaceModPath, 'Vars', stateSpaceModInputVars, 'Outputs', {'dStates'}, 'Comments', stateSpaceModFuncHeader);
                end                
                % Inverse dynamics of robot: jointTorq = M*d2q + C(q, dq)*dq + G(q) + transpose(J(q))*Fext
                jointTorq = M*obj.robot.symAccelerationVariable + C*velocities + G;            % Here, the external effort is not considered yet - should be added in futur
                sysIDPath = fullfile(matFuncRootPath, 'getInverseDynamics.m');
                sysIDInputVars = {[M, C, G], obj.robot.symStateVariable, obj.robot.symAccelerationVariable};
                sysIDFuncHeader = {'======================================================================%';...
                              ' Description: This function gives the inverse(backward) dynamics of robot';...
                              ' jointTorq = getInverseDynamics([M, C, G], states, accelerations)';...
                              ' @Inputs:';...
                              ' M: Inertia matrix of the dynamical system';...
                              ' C: Coriolis matrix of the dynamical system';...
                              ' G: Gravity matrix of the dynamical system';...
                              ' states: State variable of the dynamical system - column vector';...
                              ' accelerations: Accelerations of the dynamical system - column vector';...                              
                              ' @Output:';...
                              ' jointTorq: Torques of joint variables of the dynamical system';...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                if sparseMatrix
                    matlabFunction(jointTorq, 'File', sysIDPath, 'Vars', sysIDInputVars, 'Outputs', {'jointTorq'}, 'Comments', sysIDFuncHeader, 'Sparse', true);
                else
                    matlabFunction(jointTorq, 'File', sysIDPath, 'Vars', sysIDInputVars, 'Outputs', {'jointTorq'}, 'Comments', sysIDFuncHeader);
                end
                % Forward dynamics of robot: jointAccel = inv(M)*(u - transpose(J(q))*Fext - C(q, dq)*dq - G(q))
                jointAccel = M\(obj.robot.symControlVariable - C*velocities - G);            % Here, the external effort is not considered yet - should be added in futur
                sysFDPath = fullfile(matFuncRootPath, 'getForwardDynamics.m');
                sysFDInputVars = {[M, C, G], obj.robot.symStateVariable, obj.robot.symControlVariable};
                sysFDFuncHeader = {'======================================================================%';...
                              ' Description: This function gives the forward(direct) dynamics of robot';...
                              ' jointAccel = getForwardDynamics([M, C, G], states, controls)';...
                              ' @Inputs:';...
                              ' M: Inertia matrix of the dynamical system';...
                              ' C: Coriolis matrix of the dynamical system';...
                              ' G: Gravity matrix of the dynamical system';...
                              ' states: State variable of the dynamical system - column vector';...
                              ' controls: Control inputs of the dynamical system - column vector';...                              
                              ' @Output:';...
                              ' jointAccel: Accelerations of joint variables of the dynamical system';...
                              ' Version: 1.0';...
                              ' Author: Quoc-Viet Dang';...
                              '======================================================================%';...
                              };
                if sparseMatrix
                    matlabFunction(jointAccel, 'File', sysFDPath, 'Vars', sysFDInputVars, 'Outputs', {'jointAccel'}, 'Comments', sysFDFuncHeader, 'Sparse', true);
                else
                    matlabFunction(jointAccel, 'File', sysFDPath, 'Vars', sysFDInputVars, 'Outputs', {'jointAccel'}, 'Comments', sysFDFuncHeader); 
                end
            end
        end
        
        %*****************************************************************%
        %% getNewtonEulerRoModTool
        function getNewtonEulerRoModTool(obj)
            
        end        
    end
end
