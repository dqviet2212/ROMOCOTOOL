%*************************************************************************%
%=========================================================================%
% Project: ROMOCOTOOL
% Name: RoModTool.m
% Type: matlab object
% Version: 1.0
% Description: This object class provides methods for the robot modelling
% Author: Quoc-Viet DANG
%=========================================================================%
classdef RoModTool < handle
    properties
        Ek;             % Kinematic energy of the dynamical system - symbolic expression
        Ep;             % Potential energy of the dynamical system - symbolic expression
        states;         % State variable of the dynamical system - symbolic column vector
        accelerations;  % Accelerations of joint variables of the dynamical system - symbolic column vector
        controls;       % Control inputs of the dynamical system - symbolic column vector
        sysParams;      % Physical parameters of the dynamical system - symbolic column vector
        robotStr;
    end
    
    methods
        function obj = RoModTool(Ek, Ep, states, accelerations, controls, sysParams)
            nStates = length(states);
            nAccelerations = length(accelerations);
            nControls = length(controls);
            nSysParams = length(sysParams);
            nJoints = 0.5*nStates;
            if ~iscolumn(states)
                error('State variable should be a symbolic column vector');
            end
            if (nStates~=length(states(states~=0)))
                error('State variable should not contain zero component');
            end
            if ~iscolumn(accelerations)
                error('Accelerations should be a column vector');
            end
            if (nJoints ~= nAccelerations)
                error('The dimension of state variable should be equal 2 times that of accelerations');
            end
            if ~iscolumn(controls)
                error('Controls should be a column vector');
            end
            if (nJoints ~= nControls)
                error('The dimension of state variable should be equal 2 times that of controls');
            end
            if ~iscolumn(sysParams)
                error('Physical parameters of the system should be a symbolic column vector');
            end
            if (nSysParams~=length(sysParams(sysParams~=0)))
                error('Physical parameters of the system should not contain zero component');
            end
            %
            obj.Ek = Ek;
            obj.Ep = Ep;
            obj.states = states;
            obj.accelerations = accelerations;
            obj.controls = controls;
            obj.sysParams = sysParams;
            obj.robotStr = robotStr;
            obj.nJoints = nJoints;
        end        
        
        function robotStr = getRobotStructure(obj)
            [jointAxes, jointTypes] = getJointAxesDef();
            if (size(jointAxes, 2) == nJoints)        
                for i1 = nJoints:-1:1
                    joint(i1).name = ['joint', num2str(i1)];
                    joint(i1).axis = jointAxes(:, i1);
                    joint(i1).type = jointTypes(i1);            
                    joint(i1).origin.xyz = str2sym([['j', num2str(i1), 'x']; ['j', num2str(i1), 'y']; ['j', num2str(i1), 'z']]);
                    joint(i1).origin.rpy = str2sym([['j', num2str(i1), 'r']; ['j', num2str(i1), 'p']; ['j', num2str(i1), 'y']]);
                    joint(i1).parentlink = ['link', num2str(i1-1)];
                    joint(i1).childlink = ['link', num2str(i1)];
                    %
                    link(i1).name = ['link', num2str(i1)];
                    link(i1).origin.xyz = str2sym([['l', num2str(i1), 'x']; ['l', num2str(i1), 'y']; ['l', num2str(i1), 'z']]);
                    link(i1).origin.rpy = str2sym([['l', num2str(i1), 'r']; ['l', num2str(i1), 'p']; ['l', num2str(i1), 'y']]);
                    link(i1).mass = str2sym(['m', num2str(i1)]);
                    link(i1).inertia(1, 1) = str2sym(['ixx', num2str(i1)]);
                    link(i1).inertia(1, 2) = str2sym(['ixy', num2str(i1)]);
                    link(i1).inertia(1, 3) = str2sym(['ixz', num2str(i1)]);
                    link(i1).inertia(2, 1) = str2sym(['ixy', num2str(i1)]);
                    link(i1).inertia(2, 2) = str2sym(['iyy', num2str(i1)]);
                    link(i1).inertia(2, 3) = str2sym(['iyz', num2str(i1)]);
                    link(i1).inertia(3, 1) = str2sym(['ixz', num2str(i1)]);
                    link(i1).inertia(3, 2) = str2sym(['iyz', num2str(i1)]);
                    link(i1).inertia(3, 3) = str2sym(['izz', num2str(i1)]);
                end
                robotStr.joint = joint;
                robotStr.link = link;
            else
                error('Number of axes should be equal to number of joints');
            end        
        end
        
        function getForwardKinematics()
            
        end
        
        function [Ek, Ep] = getSystemEnergy(obj)
            nJoints = obj.robotStr.nJoints;            
            for i1 = 1:nJoints
                Jc = robotStr.joint(i1).axis'*robotStr.link(i1).inertia*robotStr.joint(i1).axis;                
                Ja(i1) = Jc + robotStr.link(i1).mass*lLink(i1).^2;
            end
            
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
            
        end        
        
        
        %=================================================================%
        % Description: This function is an implementation of the Euler-Lagrange method
        % to compute the dynamical matrices of a system via the energy-based approach.
        % @Inputs:
        % obj: Class object - symbolic object
        %   Ek: Kinematic energy of the dynamical system - symbolic expression
        %   Ep: Potential energy of the dynamical system - symbolic expression
        %   states: State variable of the dynamical system - symbolic column vector
        %   accelerations: Accelerations of joint variables of the dynamical system - symbolic column vector
        %   controls: Control inputs of the dynamical system - symbolic column vector
        %   sysParams: Physical parameters of the dynamical system - symbolic column vector
        % @Output:
        % sysDyn: System dynamics - symbolic object
        %   sysDyn.nStates: Number of state variable - positive integer
        %   sysDyn.nControls: Number of active controls - positive integer
        %   sysDyn.activeConInd: Index of active controls - positive integer
        %   sysDyn.passiveConInd: Index of passive controls - scalar column vector
        %   sysDyn.M: Inertia matrix of the dynamical system - scalar column vector
        %   sysDyn.C: Coriolis matrix of the dynamical system - symbolic matrix
        %   sysDyn.G: Gravity matrix of the dynamical system - symbolic column vector        
        %=================================================================%
        function sysDyn = getSystemDynamics(obj)
            %% System dynamics            
            nStates = length(obj.states);
            nJoints = 0.5*nStates;
            positions = obj.states(1:nJoints, 1);
            velocities = obj.states((nJoints+1):end, 1);

            %% Dynamical matrices
            % Inertia matrix
            M = sym(zeros(nJoints, nJoints));
            for i1 = 1:nJoints
                for j1 = 1:nJoints
                    M(i1, j1) = simplify(diff(diff(obj.Ek,velocities(j1, 1)),velocities(i1, 1)));
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
                    C(i1, j1) = simplify(C4);
                end
            end
            % Gravity matrix
            G = sym(zeros(nJoints, 1));
            for i1 = 1:nJoints
                G(i1, 1) = simplify(diff(obj.Ep, positions(i1)));
            end    

            %% Verification
            checkM = sym(zeros(nJoints, nJoints));
            % Symmetric property of the initia matrix, i.e. M(i, j) - M(j, i) = 0
            symM = sym(zeros(nJoints, nJoints));
            for i1 = 1:nJoints
                for j1 = 1:nJoints
                    symM(i1, j1) = simplify(M(i1, j1) - M(j1, i1));
                end
            end    
            % Anti-symmetric propety of the matrix aM = diff(M, t) - 2C, i.e. aM(i, j) + aM(j, i) = 0
            positionsT = sym(zeros(nJoints, 1));
            for i1 = 1:nJoints
                positionsT(i1, 1) = str2sym([char(positions(i1, 1)), '(t)']);
            end
            t = sym('t');
            velocitiesT = diff(positionsT, t);
            aM = sym(zeros(nJoints, nJoints));
            for i1 = 1:nJoints
                for j1 = 1:nJoints
                    aM(i1, j1) = subs(subs(diff(subs(M(i1, j1), positions, positionsT), t), velocitiesT, velocities), positionsT, positions) - 2*C(i1,j1);
                end
            end
            asymM = sym(zeros(nJoints, nJoints));
            for i1 = 1:nJoints
                for j1 = 1:nJoints
                    asymM(i1, j1) = simplify(aM(i1, j1) + aM(j1, i1));
                end
            end
            passCond1 = isequaln(checkM, symM);
            passCond2 = isequaln(checkM, asymM);
            if (passCond1&&passCond2)                
                activeConInd = find(obj.controls~=0);
                if isempty(activeConInd)
                    activeConInd = 0;
                end
                passiveConInd = find(obj.controls==0);
                if isempty(passiveConInd)
                    passiveConInd = 0;
                end
                nControls = length(activeConInd);
                %
                sysDyn.nStates = nStates;
                sysDyn.nControls = nControls;
                sysDyn.activeConInd = activeConInd;
                sysDyn.passiveConInd = passiveConInd;                
                sysDyn.M = M;
                sysDyn.C = C;
                sysDyn.G = G;
            else
                if ~passCond1
                    error('Inertia matrix is not symmetric');
                end
                if ~passCond2
                    error('Anti-symmetric property is not satisfied');    
                end        
            end            
        end
        %=================================================================%
        % Description: This function converts the system dynamics to
        % the matlab functions
        % @Inputs:
        % obj: Class object - symbolic object
        % matFuncRootPath: Matlab function root path - char type
        % sysDyn: System dynamics - symbolic object        
        % @Output:
        % sysDynMatPath: Matlab function path of system dynamics matrices - char type
        % sysDynPath: Matlab function path of system dynamics - char type
        %=================================================================%
        function [sysDynMatPath, ssModPath, sysIDPath, sysFDPath] = sysDyn2MatFunc(obj, matFuncRootPath, sysDyn)
            if ~ischar(matFuncRootPath)
                error('The Matlab function path should be char type');
            end
            tmpControls = obj.controls;
            if isnumeric(tmpControls)
                tmpControls = sym(tmpControls);
            end
            if (sysDyn.passiveConInd ~= 0)
                tmpControls(sysDyn.passiveConInd) = sym('virCon');
            end
            
            %% System dynamics matrices
            sysDynMatPath = fullfile(matFuncRootPath, 'getSysDynMat.m');
            sysDynMatInputVars = {obj.states, obj.sysParams};
            sysDynMatFuncHeader = {'======================================================================%';...
                          ' Description: This function computes the inertia, Coriolis and gravity matrices of the dynamical system';...
                          ' [M, C, G] = getSysDynMat(states, sysParams)';...
                          ' @Inputs:';...
                          ' states: State variable of the dynamical system - column vector';...
                          ' sysParams: Physical parameters of the dynamical system - column vector';...
                          ' @Output:';...
                          ' M: Inertia matrix of the dynamical system';...
                          ' C: Coriolis matrix of the dynamical system';...
                          ' G: Gravity matrix of the dynamical system';...
                          ' Version: 1.0';...
                          ' Author: Quoc-Viet Dang';...
                          '======================================================================%';...
                          };  
            matlabFunction(sysDyn.M, sysDyn.C, sysDyn.G, 'File', sysDynMatPath, 'Vars', sysDynMatInputVars, 'Outputs', {'M', 'C', 'G'}, 'Sparse', true, 'Comments', sysDynMatFuncHeader);
            
            %% State-space model
            nStates = length(obj.states);            
            nJoints = 0.5*nStates;
            velocities = obj.states((nJoints+1):end, 1);
            M = sym('M', [nJoints, nJoints]); C = sym('C', [nJoints, nJoints]); G = sym('G', [nJoints, 1]);
            dStates = [velocities; M\(obj.controls - C*velocities - G)];
            ssModPath = fullfile(matFuncRootPath, 'getStateSpaceModel.m');
            ssModInputVars = {obj.states, tmpControls, [M, C, G]};
            ssModFuncHeader = {'======================================================================%';...
                          ' Description: This function gives the state-space model of the dynamical system';...
                          ' dStates = getStateSpaceModel(states, controls, [M, C, G])';...
                          ' @Inputs:';...
                          ' states: State variable of the dynamical system - column vector';...
                          ' controls: Control inputs of the dynamical system - column vector';...
                          ' M: Inertia matrix of the dynamical system';...
                          ' C: Coriolis matrix of the dynamical system';...
                          ' G: Gravity matrix of the dynamical system';...
                          ' @Output:';...
                          ' dStates: Time derivative of state variable of the dynamical system';...
                          ' Version: 1.0';...
                          ' Author: Quoc-Viet Dang';...
                          '======================================================================%';...
                          };  
            matlabFunction(dStates, 'File', ssModPath, 'Vars', ssModInputVars, 'Outputs', {'dStates'}, 'Sparse', true, 'Comments', ssModFuncHeader);
            
            %% Inverse dynamics of system: jointTorq = M*d2q + C(q, dq)*dq + G(q) + transpose(J(q))*Fext
            jointTorq = M*obj.accelerations + C*velocities + G;            % Here, the external effort is not considered yet - should be added in futur
            sysIDPath = fullfile(matFuncRootPath, 'getInverseDynamics.m');
            sysIDInputVars = {obj.states, obj.accelerations, [M, C, G]};
            sysIDFuncHeader = {'======================================================================%';...
                          ' Description: This function gives the inverse(backward) dynamics of system';...
                          ' jointTorq = getInverseDynamics(states, accelerations, [M, C, G])';...
                          ' @Inputs:';...
                          ' states: State variable of the dynamical system - column vector';...
                          ' accelerations: Accelerations of the dynamical system - column vector';...
                          ' M: Inertia matrix of the dynamical system';...
                          ' C: Coriolis matrix of the dynamical system';...
                          ' G: Gravity matrix of the dynamical system';...
                          ' @Output:';...
                          ' jointTorq: Torques of joint variables of the dynamical system';...
                          ' Version: 1.0';...
                          ' Author: Quoc-Viet Dang';...
                          '======================================================================%';...
                          };  
            matlabFunction(jointTorq, 'File', sysIDPath, 'Vars', sysIDInputVars, 'Outputs', {'jointTorq'}, 'Sparse', true, 'Comments', sysIDFuncHeader);
            
            %% Forward dynamics of system: jointAccel = inv(M)*(u - transpose(J(q))*Fext - C(q, dq)*dq - G(q))
            jointAccel = M\(obj.controls - C*velocities - G);            % Here, the external effort is not considered yet - should be added in futur
            sysFDPath = fullfile(matFuncRootPath, 'getForwardDynamics.m');
            sysFDInputVars = {obj.states, tmpControls, [M, C, G]};
            sysFDFuncHeader = {'======================================================================%';...
                          ' Description: This function gives the forward(direct) dynamics of system';...
                          ' jointAccel = getForwardDynamics(states, controls, [M, C, G])';...
                          ' @Inputs:';...
                          ' states: State variable of the dynamical system - column vector';...
                          ' controls: Control inputs of the dynamical system - column vector';...
                          ' M: Inertia matrix of the dynamical system';...
                          ' C: Coriolis matrix of the dynamical system';...
                          ' G: Gravity matrix of the dynamical system';...
                          ' @Output:';...
                          ' jointAccel: Accelerations of joint variables of the dynamical system';...
                          ' Version: 1.0';...
                          ' Author: Quoc-Viet Dang';...
                          '======================================================================%';...
                          };  
            matlabFunction(jointAccel, 'File', sysFDPath, 'Vars', sysFDInputVars, 'Outputs', {'jointAccel'}, 'Sparse', true, 'Comments', sysFDFuncHeader);            
        end
    end
end
