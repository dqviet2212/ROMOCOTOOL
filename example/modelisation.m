clearvars; close all; clc
syms g
syms m1 l1 L1    % Thigh
syms m2 l2 L2    % Shank
syms m3 l3 L3    % Foot
syms q1 dq1 d2q1
syms q2 dq2 d2q2
syms q3 dq3 d2q3

%% Thigh
J1 = (1/12)*m1*L1^2 + m1*(0.5*L1 - l1)^2;
x1 = @(q1)(l1*sin(q1));
z1 = @(q1)(l1*cos(q1));
dx1 = diff(x1, q1)*dq1;
dz1 = diff(z1, q1)*dq1;
v1 = dx1^2 + dz1^2;
h1 = l1*cos(q1);
Ek1 = 0.5*m1*v1^2 + 0.5*J1*dq1^2;
Ep1 = m1*g*h1;

%% Shank
J2 = (1/12)*m2*L2^2 + m2*(0.5*L2 - l2)^2;
x2 = @(q1, q2)(L1*sin(q1) + l2*sin(q1 + q2));
z2 = @(q1, q2)(L1*cos(q1) + l2*cos(q1 + q2));
dx2 = diff(x2, q1)*dq1 + diff(x2, q2)*dq2;
dz2 = diff(z2, q1)*dq1 + diff(z2, q2)*dq2;
v2 = dx2^2 + dz2^2;
h2 = L1*cos(q1) + l2*cos(q1 + q2);
Ek2 = 0.5*m2*v2^2 + 0.5*J2*dq2^2;
Ep2 = m2*g*h2;

%% Foot
J3 = (1/12)*m3*L3^2 + m3*(0.5*L3 - l3)^2;
x3 = @(q1, q2, q3)(L1*sin(q1) + L2*sin(q1 + q2) + l3*cos(q1 + q2 +q3));
z3 = @(q1, q2, q3)(L1*cos(q1) + L2*cos(q1 + q2) - l3*sin(q1 + q2 + q3));
dx3 = diff(x3, q1)*dq1 + diff(x3, q2)*dq2 + diff(x3, q3)*dq3;
dz3 = diff(z3, q1)*dq1 + diff(z3, q2)*dq2 + diff(z3, q3)*dq3;
v3 = dx3^2 + dz3^2;
h3 = L1*cos(q1) + L2*cos(q1 + q2) - L3*sin(q1 + q2 + q3);
Ek3 = 0.5*m3*v3^2 + 0.5*J3*dq3^2;
Ep3 = m3*g*h3;

%% Euler-Lagrange equations
Ek = Ek1 + Ek2 + Ek3;
Ep = Ep1 + Ep2 + Ep3;
La = Ek + Ep;
% q1
dLa_q1 = diff(La, q1);
dLa_dq1 = diff(La, dq1);
u1 = diff(dLa_dq1, dq1)*d2q1 + diff(dLa_dq1, dq2)*d2q2 + diff(dLa_dq1, dq3)*d2q3 - dLa_q1;
% q2
dLa_q2 = diff(La, q2);
dLa_dq2 = diff(La, dq2);
u2 = diff(dLa_dq2, dq1)*d2q1 + diff(dLa_dq2, dq2)*d2q2 + diff(dLa_dq2, dq3)*d2q3 - dLa_q2;
% q3
dLa_q3 = diff(La, q3);
dLa_dq3 = diff(La, dq3);
u3 = diff(dLa_dq3, dq1)*d2q1 + diff(dLa_dq3, dq2)*d2q2 + diff(dLa_dq3, dq3)*d2q3 - dLa_q3;





