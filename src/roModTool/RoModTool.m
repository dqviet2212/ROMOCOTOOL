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
        % sysDynMod: System dynamics model - symbolic object
        %   sysDynMod.nStates: Number of state variable - positive integer
        %   sysDynMod.nControls: Number of active controls - positive integer
        %   sysDynMod.activeConInd: Index of active controls - positive integer
        %   sysDynMod.passiveConInd: Index of passive controls - scalar column vector
        %   sysDynMod.M: Inertia matrix of the dynamical system - scalar column vector
        %   sysDynMod.C: Coriolis matrix of the dynamical system - symbolic matrix
        %   sysDynMod.G: Gravity matrix of the dynamical system - symbolic column vector        
        %=================================================================%
        function sysDynMod = getSysDynMod(obj)
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
                sysDynMod.nStates = nStates;
                sysDynMod.nControls = nControls;
                sysDynMod.activeConInd = activeConInd;
                sysDynMod.passiveConInd = passiveConInd;                
                sysDynMod.M = M;
                sysDynMod.C = C;
                sysDynMod.G = G;
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
        % Description: This function converts the system dynamics model to
        % the matlab functions
        % @Inputs:
        % obj: Class object - symbolic object
        % matFuncRootPath: Matlab function root path - char type
        % sysDynamicsMod: System dynamics model - symbolic object        
        % @Output:
        % sysDynMatPath: Matlab function path of system dynamics matrices - char type
        % sysDynPath: Matlab function path of system dynamics - char type
        %=================================================================%
        function [sysDynMatPath, ssModPath, sysIDPath, sysFDPath] = getSysDynMod2MatFunc(obj, matFuncRootPath, sysDynMod)
            if ~ischar(matFuncRootPath)
                error('The Matlab function path should be char type');
            end
            tmpControls = obj.controls;
            if isnumeric(tmpControls)
                tmpControls = sym(tmpControls);
            end
            if (sysDynMod.passiveConInd ~= 0)
                tmpControls(sysDynMod.passiveConInd) = sym('virCon');
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
            matlabFunction(sysDynMod.M, sysDynMod.C, sysDynMod.G, 'File', sysDynMatPath, 'Vars', sysDynMatInputVars, 'Outputs', {'M', 'C', 'G'}, 'Sparse', true, 'Comments', sysDynMatFuncHeader);
            
            %% State-space model
            nStates = length(obj.states);            
            nJoints = 0.5*nStates;
            velocities = obj.states((nJoints+1):end, 1);
            M = sym('M', [nJoints, nJoints]); C = sym('C', [nJoints, nJoints]); G = sym('G', [nJoints, 1]);
            dStates = [velocities; M\(obj.controls - C*velocities - G)];
            ssModPath = fullfile(matFuncRootPath, 'getSysDyn2SSMod.m');
            ssModInputVars = {obj.states, tmpControls, [M, C, G]};
            ssModFuncHeader = {'======================================================================%';...
                          ' Description: This function gives the state-space model of the dynamical system';...
                          ' dStates = getSysDyn2SSMod(states, controls, [M, C, G])';...
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
            
            %% Inverse dynamics of system: torques = M*d2q + C(q, dq)*dq + G(q) + transpose(J(q))*Fext
            torques = M*obj.accelerations + C*velocities + G;            % Here, the external effort is not considered yet - should be added in futur
            sysIDPath = fullfile(matFuncRootPath, 'getSysDyn2ID.m');
            sysIDInputVars = {obj.states, obj.accelerations, [M, C, G]};
            sysIDFuncHeader = {'======================================================================%';...
                          ' Description: This function gives the inverse(backward) dynamics of system';...
                          ' torques = getSysDyn2ID(states, accelerations, [M, C, G])';...
                          ' @Inputs:';...
                          ' states: State variable of the dynamical system - column vector';...
                          ' accelerations: Accelerations of the dynamical system - column vector';...
                          ' M: Inertia matrix of the dynamical system';...
                          ' C: Coriolis matrix of the dynamical system';...
                          ' G: Gravity matrix of the dynamical system';...
                          ' @Output:';...
                          ' torques: Torques of joint variables of the dynamical system';...
                          ' Version: 1.0';...
                          ' Author: Quoc-Viet Dang';...
                          '======================================================================%';...
                          };  
            matlabFunction(torques, 'File', sysIDPath, 'Vars', sysIDInputVars, 'Outputs', {'torques'}, 'Sparse', true, 'Comments', sysIDFuncHeader);
            
            %% Forward dynamics of system: d2q = inv(M)*(u - transpose(J(q))*Fext - C(q, dq)*dq - G(q))
            d2q = M\(obj.controls - C*velocities - G);            % Here, the external effort is not considered yet - should be added in futur
            sysFDPath = fullfile(matFuncRootPath, 'getSysDyn2FD.m');
            sysFDInputVars = {obj.states, tmpControls, [M, C, G]};
            sysFDFuncHeader = {'======================================================================%';...
                          ' Description: This function gives the forward(direct) dynamics of system';...
                          ' accelerations = getSysDyn2FD(states, controls, [M, C, G])';...
                          ' @Inputs:';...
                          ' states: State variable of the dynamical system - column vector';...
                          ' controls: Control inputs of the dynamical system - column vector';...
                          ' M: Inertia matrix of the dynamical system';...
                          ' C: Coriolis matrix of the dynamical system';...
                          ' G: Gravity matrix of the dynamical system';...
                          ' @Output:';...
                          ' d2q: Accelerations of joint variables of the dynamical system';...
                          ' Version: 1.0';...
                          ' Author: Quoc-Viet Dang';...
                          '======================================================================%';...
                          };  
            matlabFunction(d2q, 'File', sysFDPath, 'Vars', sysFDInputVars, 'Outputs', {'accelerations'}, 'Sparse', true, 'Comments', sysFDFuncHeader);            
        end
    end
end
