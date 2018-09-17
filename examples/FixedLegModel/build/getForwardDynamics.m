function jointAccel = getForwardDynamics(in1,in2,in3)
%GETFORWARDDYNAMICS
%    JOINTACCEL = GETFORWARDDYNAMICS(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    17-Sep-2018 10:55:12

%======================================================================%
% Description: This function gives the forward(direct) dynamics of system
% jointAccel = getForwardDynamics(states, controls, [M, C, G])
% @Inputs:
% states: State variable of the dynamical system - column vector
% controls: Control inputs of the dynamical system - column vector
% M: Inertia matrix of the dynamical system
% C: Coriolis matrix of the dynamical system
% G: Gravity matrix of the dynamical system
% @Output:
% jointAccel: Accelerations of joint variables of the dynamical system
% Version: 1.0
% Author: Quoc-Viet Dang
%======================================================================%
C1_1 = in3(10);
C1_2 = in3(13);
C1_3 = in3(16);
C2_1 = in3(11);
C2_2 = in3(14);
C2_3 = in3(17);
C3_1 = in3(12);
C3_2 = in3(15);
C3_3 = in3(18);
G1 = in3(19);
G2 = in3(20);
G3 = in3(21);
M1_1 = in3(1);
M1_2 = in3(4);
M1_3 = in3(7);
M2_1 = in3(2);
M2_2 = in3(5);
M2_3 = in3(8);
M3_1 = in3(3);
M3_2 = in3(6);
M3_3 = in3(9);
dq1 = in1(4,:);
dq2 = in1(5,:);
dq3 = in1(6,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
t2 = sparse(M1_1.*M2_2.*M3_3);
t3 = sparse(M1_2.*M2_3.*M3_1);
t4 = sparse(M1_3.*M2_1.*M3_2);
t7 = sparse(M1_1.*M2_3.*M3_2);
t8 = sparse(M1_2.*M2_1.*M3_3);
t9 = sparse(M1_3.*M2_2.*M3_1);
t5 = sparse(t2+t3+t4-t7-t8-t9);
t6 = sparse(1.0./t5);
jointAccel = sparse([1,2,3],[1,1,1],[-t6.*(G3.*M1_2.*M2_3-G3.*M1_3.*M2_2-G2.*M1_2.*M3_3+G2.*M1_3.*M3_2+G1.*M2_2.*M3_3-G1.*M2_3.*M3_2-M1_2.*M2_3.*u3+M1_3.*M2_2.*u3+M1_2.*M3_3.*u2-M1_3.*M3_2.*u2-M2_2.*M3_3.*u1+M2_3.*M3_2.*u1+C1_1.*M2_2.*M3_3.*dq1-C1_1.*M2_3.*M3_2.*dq1-C2_1.*M1_2.*M3_3.*dq1+C2_1.*M1_3.*M3_2.*dq1+C3_1.*M1_2.*M2_3.*dq1-C3_1.*M1_3.*M2_2.*dq1+C1_2.*M2_2.*M3_3.*dq2-C1_2.*M2_3.*M3_2.*dq2-C2_2.*M1_2.*M3_3.*dq2+C2_2.*M1_3.*M3_2.*dq2+C3_2.*M1_2.*M2_3.*dq2-C3_2.*M1_3.*M2_2.*dq2+C1_3.*M2_2.*M3_3.*dq3-C1_3.*M2_3.*M3_2.*dq3-C2_3.*M1_2.*M3_3.*dq3+C2_3.*M1_3.*M3_2.*dq3+C3_3.*M1_2.*M2_3.*dq3-C3_3.*M1_3.*M2_2.*dq3),t6.*(G3.*M1_1.*M2_3-G3.*M1_3.*M2_1-G2.*M1_1.*M3_3+G2.*M1_3.*M3_1+G1.*M2_1.*M3_3-G1.*M2_3.*M3_1-M1_1.*M2_3.*u3+M1_3.*M2_1.*u3+M1_1.*M3_3.*u2-M1_3.*M3_1.*u2-M2_1.*M3_3.*u1+M2_3.*M3_1.*u1+C1_1.*M2_1.*M3_3.*dq1-C1_1.*M2_3.*M3_1.*dq1-C2_1.*M1_1.*M3_3.*dq1+C2_1.*M1_3.*M3_1.*dq1+C3_1.*M1_1.*M2_3.*dq1-C3_1.*M1_3.*M2_1.*dq1+C1_2.*M2_1.*M3_3.*dq2-C1_2.*M2_3.*M3_1.*dq2-C2_2.*M1_1.*M3_3.*dq2+C2_2.*M1_3.*M3_1.*dq2+C3_2.*M1_1.*M2_3.*dq2-C3_2.*M1_3.*M2_1.*dq2+C1_3.*M2_1.*M3_3.*dq3-C1_3.*M2_3.*M3_1.*dq3-C2_3.*M1_1.*M3_3.*dq3+C2_3.*M1_3.*M3_1.*dq3+C3_3.*M1_1.*M2_3.*dq3-C3_3.*M1_3.*M2_1.*dq3),-t6.*(G3.*M1_1.*M2_2-G3.*M1_2.*M2_1-G2.*M1_1.*M3_2+G2.*M1_2.*M3_1+G1.*M2_1.*M3_2-G1.*M2_2.*M3_1-M1_1.*M2_2.*u3+M1_2.*M2_1.*u3+M1_1.*M3_2.*u2-M1_2.*M3_1.*u2-M2_1.*M3_2.*u1+M2_2.*M3_1.*u1+C1_1.*M2_1.*M3_2.*dq1-C1_1.*M2_2.*M3_1.*dq1-C2_1.*M1_1.*M3_2.*dq1+C2_1.*M1_2.*M3_1.*dq1+C3_1.*M1_1.*M2_2.*dq1-C3_1.*M1_2.*M2_1.*dq1+C1_2.*M2_1.*M3_2.*dq2-C1_2.*M2_2.*M3_1.*dq2-C2_2.*M1_1.*M3_2.*dq2+C2_2.*M1_2.*M3_1.*dq2+C3_2.*M1_1.*M2_2.*dq2-C3_2.*M1_2.*M2_1.*dq2+C1_3.*M2_1.*M3_2.*dq3-C1_3.*M2_2.*M3_1.*dq3-C2_3.*M1_1.*M3_2.*dq3+C2_3.*M1_2.*M3_1.*dq3+C3_3.*M1_1.*M2_2.*dq3-C3_3.*M1_2.*M2_1.*dq3)],3,1);
