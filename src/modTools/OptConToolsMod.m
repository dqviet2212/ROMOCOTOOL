%*************************************************************************%
%=========================================================================%
% Author: Quoc-Viet DANG
% Project: optConTools
% Name: optConToolsMod.m
% Type: matlab object
% Version: 08 September 2018
% Description: This object class provides methods to compute the system dynamics model
%=========================================================================%
classdef OptConToolsMod < handle
    properties
        Ek;         % Kinematic energy of the dynamical system - symbolic expression
        Ep;         % Potential energy of the dynamical system - symbolic expression
        states;     % State variable of the dynamical system - symbolic column vector
        controls;   % Control inputs of the dynamical system - symbolic column vector
        sysParams;  % Physical parameters of the dynamical system - symbolic column vector
    end
    
    methods
        function obj = OptConToolsMod(Ek, Ep, states, controls, sysParams)
            nStates = length(states);
            nControls = length(controls);
            nSysParams = length(sysParams);
            nJoints = 0.5*nStates;
            if ~iscolumn(states)
                error('State variable should be a symbolic column vector');
            end
            if (nStates~=length(states(states~=0)))
                error('State variable should not contain zero component');
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
        %   controls: Control inputs of the dynamical system - symbolic column vector
        %   sysParams: Physical parameters of the dynamical system - symbolic column vector
        % @Output:
        % sysDynamicsMod: System dynamics model - symbolic object
        %   sysDynamicsMod.nStates: Number of state variable - positive integer
        %   sysDynamicsMod.nControls: Number of active controls - positive integer
        %   sysDynamicsMod.activeConInd: Index of active controls - positive integer
        %   sysDynamicsMod.passiveConInd: Index of passive controls - scalar column vector
        %   sysDynamicsMod.M: Inertia matrix of the dynamical system - scalar column vector
        %   sysDynamicsMod.C: Coriolis matrix of the dynamical system - symbolic matrix
        %   sysDynamicsMod.G: Gravity matrix of the dynamical system - symbolic column vector
        %   sysDynamicsMod.EoM: Equation of motion of the dynamical system dStates = function(states, controls, sysParams) - symbolic column vector
        %=================================================================%
        function sysDynamicsMod = getSysDynMod(obj)
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
                EoM = [velocities; M\(obj.controls - C*velocities - G)];                
                sysDynamicsMod.nStates = nStates;
                sysDynamicsMod.nControls = nControls;
                sysDynamicsMod.activeConInd = activeConInd;
                sysDynamicsMod.passiveConInd = passiveConInd;
                sysDynamicsMod.M = M;
                sysDynamicsMod.C = C;
                sysDynamicsMod.G = G;
                sysDynamicsMod.EoM = EoM;
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
        % Description: This function converts the symbolic expression EoM
        % the matlab function
        % @Inputs:
        % obj: Class object - symbolic object
        % matFuncRootPath: Matlab function root path - char type
        % sysDynamicsMod: System dynamics model - symbolic object        
        % @Output:
        % matFunctionPath: Matlab function path of system dynamics - char type     
        %=================================================================%
        function matFunctionPath = getSysDynMod2MatFunc(obj, matFuncRootPath, sysDynamicsMod)
            if ~ischar(matFuncRootPath)
                error('The Matlab function path should be char type');
            end
            matFunctionPath = fullfile(matFuncRootPath, 'getSysDynamics.m');
            funcHeader = {'======================================================================%';...
                          ' Description: This function represents the equation of motion of the dynamical system dStates = getSysDynamics(states, controls, sysParams)';...
                          ' @Inputs:';...
                          ' states: State variable of the dynamical system';...
                          ' controls: Control inputs of the dynamical system';...
                          ' sysParams: Physical parameters of the dynamical system';...
                          ' @Output:';...
                          ' dStates: Time derivative of state variable of the dynamical system';...
                          ' Version: 1.0';...
                          ' Author: Quoc-Viet Dang';...
                          '======================================================================%';...
                          };  
            tmpControls = obj.controls;
            if isnumeric(tmpControls)
                tmpControls = sym(tmpControls);
            end
            if (sysDynamicsMod.passiveConInd ~= 0)
                tmpControls(sysDynamicsMod.passiveConInd) = sym('virtualControl');
            end
            inputVars = {obj.states, tmpControls, obj.sysParams};
            matlabFunction(sysDynamicsMod.EoM, 'File', matFunctionPath, 'Vars', inputVars, 'Outputs', {'dStates'}, 'Sparse', true, 'Comments', funcHeader);
        end
    end
end
