function dStates = getSysDynamics(in1,in2,in3)
%GETSYSDYNAMICS
%    DSTATES = GETSYSDYNAMICS(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    12-Sep-2018 23:18:51

%======================================================================%
% Description: This function represents the state-space model of the dynamical system dStates = getSysDynamics(states, controls, sysParams)
% @Inputs:
% states: State variable of the dynamical system
% controls: Control inputs of the dynamical system
% sysDynMat = getSysDynMatrices(states, sysParams): Inertia, Coriolis and gravity matrices of the dynamical system
% @Output:
% dStates: Time derivative of state variable of the dynamical system
% Version: 1.0
% Author: Quoc-Viet Dang
%======================================================================%
C = in3(:,2);
G = in3(:,3);
M = in3(:,1);
dq1 = in1(4,:);
dq2 = in1(5,:);
dq3 = in1(6,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
t2 = sparse(1.0./M);
dStates = sparse([1,2,3,4,5,6],[1,1,1,1,1,1],[dq1,dq2,dq3,-t2.*(G-u1+C.*dq1),-t2.*(G-u2+C.*dq2),-t2.*(G-u3+C.*dq3)],6,1);