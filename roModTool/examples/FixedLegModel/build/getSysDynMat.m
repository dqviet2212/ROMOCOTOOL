function [M,C,G] = getSysDynMat(in1,in2)
%GETSYSDYNMAT
%    [M,C,G] = GETSYSDYNMAT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    17-Sep-2018 18:18:25

%======================================================================%
% Description: This function computes the inertia, Coriolis and gravity matrices of the dynamical system
% [M, C, G] = getSysDynMat(states, sysParams)
% @Inputs:
% states: State variable of the dynamical system - column vector
% sysParams: Physical parameters of the dynamical system - column vector
% @Output:
% M: Inertia matrix of the dynamical system
% C: Coriolis matrix of the dynamical system
% G: Gravity matrix of the dynamical system
% Version: 1.0
% Author: Quoc-Viet Dang
%======================================================================%
L1 = in2(3,:);
L2 = in2(6,:);
L3 = in2(9,:);
dq1 = in1(4,:);
dq2 = in1(5,:);
dq3 = in1(6,:);
g = in2(10,:);
l1 = in2(2,:);
l2 = in2(5,:);
l3 = in2(8,:);
m1 = in2(1,:);
m2 = in2(4,:);
m3 = in2(7,:);
q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = sparse(L1.^2);
t3 = sparse(L2.^2);
t4 = sparse(l3.^2);
t5 = sparse(cos(q2));
t6 = sparse(q2+q3);
t7 = sparse(sin(t6));
t8 = sparse(sin(q3));
t11 = sparse(dq1.*t2);
t45 = sparse(L1.*L2.*dq1.*t5.*2.0);
t46 = sparse(L1.*dq1.*l3.*t7.*2.0);
t90 = sparse(dq1.*t3);
t91 = sparse(dq2.*t3);
t92 = sparse(dq1.*t4);
t93 = sparse(dq2.*t4);
t94 = sparse(dq3.*t4);
t95 = sparse(L2.*dq1.*l3.*t8.*2.0);
t96 = sparse(L2.*dq2.*l3.*t8.*2.0);
t97 = sparse(L2.*dq3.*l3.*t8);
t137 = sparse(L1.*L2.*dq2.*t5);
t138 = sparse(L1.*dq2.*l3.*t7);
t139 = sparse(L1.*dq3.*l3.*t7);
t9 = sparse(t11+t45-t46+t90+t91+t92+t93+t94-t95-t96-t97+t137-t138-t139);
t10 = sparse(L1-l1.*2.0);
t12 = sparse(l2.^2);
t13 = sparse(t11+dq1.*t12+dq2.*t12+L1.*dq1.*l2.*t5.*2.0+L1.*dq2.*l2.*t5);
t14 = sparse(l1.^2);
t15 = sparse(q1+q2+q3);
t16 = sparse(cos(t15));
t17 = sparse(q1+q2);
t18 = sparse(sin(t17));
t25 = sparse(dq1.*l3.*t16);
t26 = sparse(dq2.*l3.*t16);
t27 = sparse(dq3.*l3.*t16);
t28 = sparse(L2.*dq1.*t18);
t29 = sparse(L2.*dq2.*t18);
t30 = sparse(sin(q1));
t31 = sparse(L1.*dq1.*t30);
t19 = sparse(t25+t26+t27+t28+t29+t31);
t20 = sparse(sin(t15));
t21 = sparse(cos(t17));
t33 = sparse(dq1.*l3.*t20);
t34 = sparse(dq2.*l3.*t20);
t35 = sparse(dq3.*l3.*t20);
t36 = sparse(L2.*dq1.*t21);
t37 = sparse(L2.*dq2.*t21);
t38 = sparse(cos(q1));
t39 = sparse(L1.*dq1.*t38);
t22 = sparse(-t33-t34-t35+t36+t37+t39);
t23 = sparse(t2.*2.0);
t24 = sparse(dq1.^2);
t32 = sparse(t19.^2);
t40 = sparse(t33+t34+t35-t36-t37-t39);
t41 = sparse(t3.*2.0);
t42 = sparse(t4.*2.0);
t43 = sparse(dq2.^2);
t44 = sparse(t5.^2);
t47 = sparse(dq1.*t3.*2.0);
t48 = sparse(dq2.*t3.*2.0);
t49 = sparse(dq1.*t4.*2.0);
t50 = sparse(dq2.*t4.*2.0);
t51 = sparse(dq3.*t4.*2.0);
t52 = sparse(t2.*t24);
t53 = sparse(t12.*t24);
t54 = sparse(t12.*t43);
t55 = sparse(dq1.*dq2.*t12.*2.0);
t56 = sparse(L1.*l2.*t5.*t24.*2.0);
t57 = sparse(L1.*dq1.*dq2.*l2.*t5.*2.0);
t58 = sparse(t52+t53+t54+t55+t56+t57);
t59 = sparse(dq1.*t2.*2.0);
t60 = sparse(L1.*L2.*dq1.*t5.*4.0);
t61 = sparse(L1.*L2.*dq2.*t5.*2.0);
t81 = sparse(L2.*dq1.*l3.*t8.*4.0);
t82 = sparse(L2.*dq2.*l3.*t8.*4.0);
t83 = sparse(L2.*dq3.*l3.*t8.*2.0);
t84 = sparse(L1.*dq1.*l3.*t7.*4.0);
t85 = sparse(L1.*dq2.*l3.*t7.*2.0);
t86 = sparse(L1.*dq3.*l3.*t7.*2.0);
t62 = sparse(t47+t48+t49+t50+t51+t59+t60+t61-t81-t82-t83-t84-t85-t86);
t63 = sparse(t40.^2);
t64 = sparse(t32+t63);
t65 = sparse(L1.*L2.*t5.*2.0);
t101 = sparse(L2.*l3.*t8.*4.0);
t66 = sparse(t41+t42+t65-t101-L1.*l3.*t7.*2.0);
t67 = sparse(m3.*t64.*t66);
t68 = sparse(l2.*t12.*t24);
t69 = sparse(l2.*t12.*t43);
t70 = sparse(l2.*t2.*t24);
t71 = sparse(L1.*t2.*t5.*t24);
t72 = sparse(dq1.*dq2.*l2.*t12.*2.0);
t73 = sparse(L1.*t5.*t12.*t24.*3.0);
t74 = sparse(L1.*t5.*t12.*t43);
t75 = sparse(dq1.*dq2.*l2.*t2);
t76 = sparse(l2.*t2.*t24.*t44.*2.0);
t77 = sparse(L1.*dq1.*dq2.*t5.*t12.*4.0);
t78 = sparse(dq1.*dq2.*l2.*t2.*t44);
t79 = sparse(t68+t69+t70+t71+t72+t73+t74+t75+t76+t77+t78);
t80 = sparse(l2.*m2.*t79.*4.0);
t87 = sparse(L1.*t5);
t88 = sparse(l2+t87);
t89 = sparse(l2.*m2.*t58.*t88.*2.0);
t98 = sparse(t90+t91+t92+t93+t94-t95-t96-t97+L1.*L2.*dq1.*t5-L1.*dq1.*l3.*t7);
t99 = sparse(L2-l2.*2.0);
t100 = sparse(dq1.*l2+dq2.*l2+L1.*dq1.*t5);
t102 = sparse(L2.*t8);
t103 = sparse(dq1.*l3);
t104 = sparse(dq2.*l3);
t105 = sparse(dq3.*l3);
t108 = sparse(L1.*dq1.*t7);
t109 = sparse(L2.*dq1.*t8);
t110 = sparse(L2.*dq2.*t8);
t106 = sparse(t103+t104+t105-t108-t109-t110);
t107 = sparse(t45-t46+t47+t48+t49+t50+t51-t81-t82-t83);
t111 = sparse(l3.*m3.*t62.*t106.*2.0);
t112 = sparse(L1.*t7);
t113 = sparse(-l3+t102+t112);
t114 = sparse(t111-l3.*m3.*t64.*t113.*2.0);
t115 = sparse(l3-t102);
t116 = sparse(l3.*m3.*t64.*t115.*2.0);
t117 = sparse(l3.*m3.*t106.*t107.*2.0);
t118 = sparse(t116+t117);
t119 = sparse(t4.^2);
t120 = sparse(q3.*2.0);
t121 = sparse(cos(t120));
t122 = sparse(q2+t120);
t123 = sparse(cos(t122));
t124 = sparse(q2.*2.0);
t125 = sparse(sin(t124));
t126 = sparse(cos(t6));
t127 = sparse(dq3.^2);
t128 = sparse(sin(q2));
t129 = sparse(t120+t124);
t130 = sparse(sin(t129));
t131 = sparse(sin(t122));
t132 = sparse(q2-q3);
t133 = sparse(cos(t132));
t134 = sparse(q3+t124);
t135 = sparse(cos(t134));
t136 = sparse(cos(q3));
t140 = sparse(L1.*L2.*t5.*4.0);
M = sparse([1,2,3,1,2,3,1,2,3],[1,1,1,2,2,2,3,3,3],[m1.*t2.*(1.0./1.2e1)+m1.*t10.^2.*(1.0./4.0)+m3.*t9.^2.*4.0+m2.*t13.^2.*4.0+m1.*t14.^2.*t24.*6.0+m2.*t58.*(t12.*2.0+t23+L1.*l2.*t5.*4.0)+m3.*(t32+t22.^2).*(t23+t41+t42+t140-L1.*l3.*t7.*4.0-L2.*l3.*t8.*4.0),t67+t80+t89+m3.*t62.*t107,t114,t67+t80+t89+m3.*t62.*(t45-t46+t47+t48+t49+t50+t51-L2.*dq1.*l3.*t8.*4.0-L2.*dq2.*l3.*t8.*4.0-L2.*dq3.*l3.*t8.*2.0),m2.*t3.*(1.0./1.2e1)+m2.*t99.^2.*(1.0./4.0)+m3.*t98.^2.*4.0+m2.*t12.*t100.^2.*4.0+m3.*t64.*(t41+t42-t101)+m2.*t12.*t58.*2.0,t118,t114,t118,m3.*(t4.*-3.0+L3.*l3.*3.0-t24.*t119.*1.8e1-t43.*t119.*1.8e1-t119.*t127.*1.8e1-L3.^2-dq1.*dq2.*t119.*3.6e1-dq1.*dq3.*t119.*3.6e1-dq2.*dq3.*t119.*3.6e1-t2.*t4.*t24.*1.2e1-t3.*t4.*t24.*1.2e1-t3.*t4.*t43.*1.2e1-dq1.*dq2.*t3.*t4.*2.4e1+t3.*t4.*t24.*t121.*6.0+t3.*t4.*t43.*t121.*6.0+t2.*t4.*t24.*cos(t129).*6.0-L1.*L2.*t4.*t5.*t24.*2.4e1+L1.*L2.*t4.*t24.*t123.*1.2e1+L1.*l3.*t4.*t7.*t24.*3.6e1+L2.*l3.*t4.*t8.*t24.*3.6e1+L2.*l3.*t4.*t8.*t43.*3.6e1+dq1.*dq2.*t3.*t4.*t121.*1.2e1-L1.*L2.*dq1.*dq2.*t4.*t5.*2.4e1+L1.*L2.*dq1.*dq2.*t4.*t123.*1.2e1+L1.*dq1.*dq2.*l3.*t4.*t7.*3.6e1+L1.*dq1.*dq3.*l3.*t4.*t7.*3.6e1+L2.*dq1.*dq2.*l3.*t4.*t8.*7.2e1+L2.*dq1.*dq3.*l3.*t4.*t8.*3.6e1+L2.*dq2.*dq3.*l3.*t4.*t8.*3.6e1).*(-1.0./3.0)],3,3);
if nargout > 1
    t141 = sparse(sin(t120));
    t142 = sparse(dq3.*m3.*t3.*t4.*t127.*t141.*2.0);
    t143 = sparse(dq1.*m3.*t2.*t4.*t24.*t130.*1.2e1);
    t144 = sparse(L1.*L2.*dq1.*m3.*t4.*t24.*t131.*1.2e1);
    t145 = sparse(L1.*L2.*dq3.*m3.*t4.*t127.*t131.*2.0);
    t146 = sparse(dq1.*m3.*t3.*t4.*t127.*t141.*1.2e1);
    t147 = sparse(dq3.*m3.*t3.*t4.*t24.*t141.*1.2e1);
    t148 = sparse(dq2.*m3.*t3.*t4.*t127.*t141.*1.2e1);
    t149 = sparse(dq3.*m3.*t3.*t4.*t43.*t141.*1.2e1);
    t150 = sparse(L1.*dq1.*l3.*m3.*t3.*t24.*t133.*1.2e1);
    t151 = sparse(dq1.*dq2.*dq3.*m3.*t3.*t4.*t141.*2.4e1);
    t152 = sparse(L1.*dq1.*l3.*m3.*t4.*t43.*t126.*6.0);
    t153 = sparse(L1.*dq1.*l3.*m3.*t4.*t126.*t127.*6.0);
    t154 = sparse(L1.*L2.*dq1.*m3.*t3.*t43.*t128.*6.0);
    t155 = sparse(L1.*dq1.*l2.*m2.*t12.*t43.*t128.*6.0);
    t156 = sparse(L1.*L2.*dq1.*m3.*t4.*t43.*t128.*1.2e1);
    t157 = sparse(L1.*dq1.*dq2.*dq3.*l3.*m3.*t4.*t126.*1.2e1);
    t158 = sparse(L1.*dq1.*l3.*m3.*t3.*t43.*t126.*1.2e1);
    t159 = sparse(L1.*dq1.*dq2.*dq3.*l3.*m3.*t3.*t133.*4.0);
    t160 = sparse(L2.*dq1.*t3.*t24.*t136.*6.0);
    t161 = sparse(L2.*dq2.*t3.*t43.*t136.*6.0);
    t162 = sparse(L2.*dq1.*t3.*t43.*t136.*1.8e1);
    t163 = sparse(L2.*dq2.*t3.*t24.*t136.*1.8e1);
    t164 = sparse(L2.*dq3.*t3.*t24.*t136.*6.0);
    t165 = sparse(L2.*dq3.*t3.*t43.*t136.*6.0);
    t166 = sparse(L2.*dq1.*t4.*t24.*t136.*6.0);
    t167 = sparse(L2.*dq2.*t4.*t43.*t136.*6.0);
    t168 = sparse(L2.*dq3.*t4.*t127.*t136.*3.0);
    t169 = sparse(L1.*dq1.*t3.*t24.*t133.*6.0);
    t170 = sparse(L2.*dq1.*dq2.*dq3.*t3.*t136.*1.2e1);
    t171 = sparse(L2.*dq1.*t4.*t43.*t136.*1.8e1);
    t172 = sparse(L2.*dq2.*t4.*t24.*t136.*1.8e1);
    t173 = sparse(L2.*dq1.*t4.*t127.*t136.*1.5e1);
    t174 = sparse(L2.*dq3.*t4.*t24.*t136.*1.8e1);
    t175 = sparse(L2.*dq2.*t4.*t127.*t136.*1.5e1);
    t176 = sparse(L2.*dq3.*t4.*t43.*t136.*1.8e1);
    t177 = sparse(L2.*dq1.*dq2.*dq3.*t4.*t136.*3.6e1);
    t178 = sparse(L1.*L2.*dq1.*dq2.*dq3.*l3.*t128.*4.0);
    t179 = sparse(L1.*dq1.*t2.*t24.*t126.*6.0);
    t180 = sparse(L2.*dq1.*t2.*t24.*t135.*6.0);
    t181 = sparse(L1.*dq1.*t3.*t24.*t126.*1.2e1);
    t182 = sparse(L1.*dq1.*t4.*t24.*t126.*6.0);
    t183 = sparse(L2.*dq1.*t2.*t24.*t136.*1.2e1);
    t184 = sparse(L1.*L2.*dq2.*l3.*t43.*t128.*2.0);
    t185 = sparse(L1.*dq1.*t4.*t43.*t126.*3.0);
    t186 = sparse(L1.*dq1.*t4.*t126.*t127.*3.0);
    t187 = sparse(L2.*dq1.*t2.*t43.*t136.*6.0);
    t188 = sparse(L2.*dq2.*t2.*t24.*t136.*1.8e1);
    t189 = sparse(L1.*dq1.*dq2.*dq3.*t3.*t126.*8.0);
    t190 = sparse(L1.*dq1.*dq2.*dq3.*t4.*t126.*6.0);
    t191 = sparse(L1.*L2.*dq1.*l3.*t43.*t128.*8.0);
    t192 = sparse(L1.*L2.*dq2.*l3.*t24.*t128.*6.0);
    t193 = sparse(L1.*L2.*dq3.*l3.*t43.*t128.*2.0);
    t194 = sparse(L1.*dq1.*dq2.*dq3.*t3.*t133.*4.0);
    t195 = sparse(L2.*dq3.*t3.*t24.*t136.*3.0);
    t196 = sparse(L2.*dq3.*t3.*t43.*t136.*3.0);
    t197 = sparse(L1.*dq1.*t3.*t24.*t126.*6.0);
    t198 = sparse(L2.*dq1.*t2.*t24.*t136.*6.0);
    t199 = sparse(L2.*dq1.*dq2.*dq3.*t3.*t136.*6.0);
    t200 = sparse(L1.*dq2.*t4.*t24.*t126.*3.0);
    t201 = sparse(L1.*dq3.*t4.*t24.*t126.*3.0);
    t202 = sparse(L2.*dq2.*t2.*t24.*t136.*6.0);
    t203 = sparse(L2.*dq1.*t4.*t127.*t136.*3.0);
    t204 = sparse(L2.*dq3.*t4.*t24.*t136.*9.0);
    t205 = sparse(L2.*dq2.*t4.*t127.*t136.*3.0);
    t206 = sparse(L2.*dq3.*t4.*t43.*t136.*9.0);
    t207 = sparse(L1.*dq1.*t3.*t43.*t133.*7.0);
    t208 = sparse(L2.*dq2.*t2.*t24.*t135);
    t209 = sparse(L2.*dq1.*dq2.*dq3.*t4.*t136.*1.8e1);
    C = sparse([1,2,3,1,2,3,1,2,3],[1,1,1,2,2,2,3,3,3],[-dq2.*(m3.*t2.*t3.*t24.*t125.*1.2e1-m3.*t2.*t4.*t24.*t130.*1.2e1+m2.*t2.*t12.*t24.*t125.*1.2e1+m3.*t2.*t3.*t43.*t125.*2.0-m3.*t2.*t4.*t43.*t130.*2.0+m2.*t2.*t12.*t43.*t125.*2.0-m3.*t2.*t4.*t127.*t130.*2.0+L1.*L2.*m3.*t2.*t24.*t128.*1.2e1+L1.*L2.*m3.*t3.*t24.*t128.*1.2e1+L1.*L2.*m3.*t4.*t24.*t128.*2.4e1-L1.*L2.*m3.*t4.*t24.*t131.*1.2e1+L1.*L2.*m3.*t3.*t43.*t128.*6.0+L1.*L2.*m3.*t4.*t43.*t128.*1.2e1-L1.*L2.*m3.*t4.*t43.*t131.*6.0+L1.*L2.*m3.*t4.*t127.*t128.*4.0-L1.*L2.*m3.*t4.*t127.*t131.*2.0+L1.*l2.*m2.*t2.*t24.*t128.*1.2e1+L1.*l3.*m3.*t2.*t24.*t126.*1.2e1+L1.*l3.*m3.*t3.*t24.*t126.*2.4e1+L1.*l3.*m3.*t4.*t24.*t126.*1.2e1-L1.*l3.*m3.*t3.*t24.*t133.*1.2e1+L1.*l2.*m2.*t12.*t24.*t128.*1.2e1+L2.*l3.*m3.*t2.*t24.*t135.*2.4e1+L1.*l3.*m3.*t3.*t43.*t126.*1.2e1+L1.*l3.*m3.*t4.*t43.*t126.*6.0-L1.*l3.*m3.*t3.*t43.*t133.*6.0+L1.*l2.*m2.*t12.*t43.*t128.*6.0+L2.*l3.*m3.*t2.*t43.*t135.*4.0+L1.*l3.*m3.*t4.*t126.*t127.*6.0+dq1.*dq2.*m3.*t2.*t3.*t125.*1.2e1-dq1.*dq2.*m3.*t2.*t4.*t130.*1.2e1-dq1.*dq3.*m3.*t2.*t4.*t130.*1.2e1+dq1.*dq2.*m2.*t2.*t12.*t125.*1.2e1-dq2.*dq3.*m3.*t2.*t4.*t130.*4.0+L1.*L2.*dq1.*dq2.*m3.*t2.*t128.*6.0+L1.*L2.*dq1.*dq2.*m3.*t3.*t128.*1.8e1+L1.*L2.*dq1.*dq2.*m3.*t4.*t128.*3.6e1+L1.*L2.*dq1.*dq3.*m3.*t4.*t128.*2.4e1+L1.*L2.*dq2.*dq3.*m3.*t4.*t128.*1.6e1-L1.*L2.*dq1.*dq2.*m3.*t4.*t131.*1.8e1-L1.*L2.*dq1.*dq3.*m3.*t4.*t131.*1.2e1-L1.*L2.*dq2.*dq3.*m3.*t4.*t131.*8.0+L1.*dq1.*dq2.*l2.*m2.*t2.*t128.*6.0+L1.*dq1.*dq2.*l3.*m3.*t2.*t126.*6.0+L1.*dq1.*dq2.*l3.*m3.*t3.*t126.*3.6e1+L1.*dq1.*dq3.*l3.*m3.*t2.*t126.*6.0+L1.*dq1.*dq2.*l3.*m3.*t4.*t126.*1.8e1+L1.*dq1.*dq3.*l3.*m3.*t3.*t126.*1.2e1+L1.*dq1.*dq3.*l3.*m3.*t4.*t126.*1.8e1+L1.*dq2.*dq3.*l3.*m3.*t3.*t126.*8.0+L1.*dq2.*dq3.*l3.*m3.*t4.*t126.*1.2e1-L1.*dq1.*dq2.*l3.*m3.*t3.*t133.*1.8e1-L1.*dq1.*dq3.*l3.*m3.*t3.*t133.*6.0+L1.*dq1.*dq2.*l2.*m2.*t12.*t128.*1.8e1-L1.*dq2.*dq3.*l3.*m3.*t3.*t133.*4.0+L2.*dq1.*dq2.*l3.*m3.*t2.*t135.*2.4e1+L2.*dq1.*dq3.*l3.*m3.*t2.*t135.*1.2e1+L2.*dq2.*dq3.*l3.*m3.*t2.*t135.*4.0)-dq3.*(l3.*m3.*t9.*(L1.*dq1.*t126.*2.0+L1.*dq2.*t126+L1.*dq3.*t126+L2.*dq1.*t136.*2.0+L2.*dq2.*t136.*2.0+L2.*dq3.*t136).*4.0+l3.*m3.*t64.*(L1.*t126+L2.*t136).*2.0+l3.*m3.*(dq1+dq2+dq3).*(L1.*dq1.*t126+L2.*dq1.*t136+L2.*dq2.*t136).*(t23+t41+t42-t101+t140-L1.*l3.*t7.*4.0)),t142-t143-t144+t145+t146+t147+t148+t149-t150+t151+t152+t153+t154+t155+t156+t157+t158+dq1.*m3.*t2.*t3.*t24.*t125.*1.2e1+dq2.*m3.*t2.*t3.*t24.*t125.*1.2e1-dq2.*m3.*t2.*t4.*t24.*t130.*1.2e1+dq1.*m2.*t2.*t12.*t24.*t125.*1.2e1-dq3.*m3.*t2.*t4.*t24.*t130.*1.2e1+dq2.*m2.*t2.*t12.*t24.*t125.*1.2e1+dq1.*m3.*t2.*t3.*t43.*t125.*2.0-dq1.*m3.*t2.*t4.*t43.*t130.*2.0+dq1.*m2.*t2.*t12.*t43.*t125.*2.0-dq1.*m3.*t2.*t4.*t127.*t130.*2.0+L1.*L2.*dq1.*m3.*t2.*t24.*t128.*1.2e1+L1.*L2.*dq1.*m3.*t3.*t24.*t128.*1.2e1+L1.*L2.*dq2.*m3.*t2.*t24.*t128.*6.0+L1.*L2.*dq1.*m3.*t4.*t24.*t128.*2.4e1+L1.*L2.*dq2.*m3.*t3.*t24.*t128.*1.8e1+L1.*L2.*dq2.*m3.*t4.*t24.*t128.*3.6e1+L1.*L2.*dq3.*m3.*t4.*t24.*t128.*3.6e1-L1.*L2.*dq2.*m3.*t4.*t24.*t131.*1.8e1+L1.*L2.*dq3.*m3.*t4.*t43.*t128.*4.0-L1.*L2.*dq1.*m3.*t4.*t43.*t131.*6.0+L1.*L2.*dq3.*m3.*t4.*t43.*t131.*4.0+L1.*L2.*dq1.*m3.*t4.*t127.*t128.*1.2e1+L1.*L2.*dq2.*m3.*t4.*t127.*t128.*4.0+L1.*L2.*dq1.*m3.*t4.*t127.*t131.*1.0e1+L1.*L2.*dq2.*m3.*t4.*t127.*t131.*6.0+L1.*dq1.*l2.*m2.*t2.*t24.*t128.*1.2e1+L1.*dq1.*l3.*m3.*t2.*t24.*t126.*1.2e1+L1.*dq1.*l3.*m3.*t3.*t24.*t126.*2.4e1+L1.*dq2.*l2.*m2.*t2.*t24.*t128.*6.0+L1.*dq2.*l3.*m3.*t2.*t24.*t126.*6.0+L1.*dq1.*l3.*m3.*t4.*t24.*t126.*1.2e1+L1.*dq2.*l3.*m3.*t3.*t24.*t126.*3.6e1+L1.*dq3.*l3.*m3.*t2.*t24.*t126.*6.0+L1.*dq2.*l3.*m3.*t4.*t24.*t126.*1.8e1+L1.*dq3.*l3.*m3.*t4.*t24.*t126.*1.8e1-L1.*dq2.*l3.*m3.*t3.*t24.*t133.*1.8e1+L1.*dq1.*l2.*m2.*t12.*t24.*t128.*1.2e1-L1.*dq3.*l3.*m3.*t3.*t24.*t133.*1.8e1+L2.*dq1.*l3.*m3.*t2.*t24.*t135.*2.4e1+L1.*dq2.*l2.*m2.*t12.*t24.*t128.*1.8e1+L2.*dq2.*l3.*m3.*t2.*t24.*t135.*2.4e1+L2.*dq3.*l3.*m3.*t2.*t24.*t135.*1.2e1-L2.*dq3.*l3.*m3.*t2.*t24.*t136.*1.2e1-L2.*dq3.*l3.*m3.*t3.*t24.*t136.*1.2e1-L2.*dq3.*l3.*m3.*t4.*t24.*t136.*1.2e1-L1.*dq3.*l3.*m3.*t3.*t43.*t126.*4.0-L1.*dq1.*l3.*m3.*t3.*t43.*t133.*6.0-L1.*dq3.*l3.*m3.*t3.*t43.*t133.*4.0+L2.*dq1.*l3.*m3.*t2.*t43.*t135.*4.0-L2.*dq3.*l3.*m3.*t3.*t43.*t136.*1.2e1-L2.*dq3.*l3.*m3.*t4.*t43.*t136.*1.2e1-L1.*dq1.*l3.*m3.*t3.*t126.*t127.*8.0-L1.*dq2.*l3.*m3.*t3.*t126.*t127.*4.0-L1.*dq1.*l3.*m3.*t3.*t127.*t133.*4.0-L1.*dq2.*l3.*m3.*t3.*t127.*t133.*2.0-L2.*dq1.*l3.*m3.*t2.*t127.*t135.*2.0-L2.*dq1.*l3.*m3.*t2.*t127.*t136.*4.0-L2.*dq1.*l3.*m3.*t3.*t127.*t136.*6.0-L2.*dq1.*l3.*m3.*t4.*t127.*t136.*1.8e1-L2.*dq2.*l3.*m3.*t3.*t127.*t136.*6.0-L2.*dq2.*l3.*m3.*t4.*t127.*t136.*1.8e1-L2.*dq3.*l3.*m3.*t4.*t127.*t136.*6.0-dq1.*dq2.*dq3.*m3.*t2.*t4.*t130.*4.0+L1.*L2.*dq1.*dq2.*dq3.*m3.*t4.*t128.*3.2e1+L1.*L2.*dq1.*dq2.*dq3.*m3.*t4.*t131.*8.0-L1.*dq1.*dq2.*dq3.*l3.*m3.*t3.*t126.*8.0-L1.*dq1.*dq2.*dq3.*l3.*m3.*t3.*t133.*2.0e1+L2.*dq1.*dq2.*dq3.*l3.*m3.*t2.*t135.*4.0-L2.*dq1.*dq2.*dq3.*l3.*m3.*t2.*t136.*8.0-L2.*dq1.*dq2.*dq3.*l3.*m3.*t3.*t136.*2.4e1-L2.*dq1.*dq2.*dq3.*l3.*m3.*t4.*t136.*2.4e1,l3.*m3.*(t160+t161+t162+t163+t166+t167+t169+t171+t172-t178+t179+t180+t181+t182+t183-t184+t185+t186+t187+t188+t189+t190-t191-t192-t193+t194+t195+t196+t199+t203+t204+t205+t206+t209+L1.*dq2.*t2.*t24.*t126.*3.0+L1.*dq2.*t3.*t24.*t126.*2.4e1+L1.*dq3.*t2.*t24.*t126.*3.0+L1.*dq2.*t4.*t24.*t126.*9.0+L1.*dq3.*t3.*t24.*t126.*6.0+L1.*dq3.*t4.*t24.*t126.*9.0+L1.*dq2.*t3.*t24.*t133.*1.5e1+L1.*dq3.*t3.*t24.*t133.*3.0+L2.*dq2.*t2.*t24.*t135.*6.0+L2.*dq3.*t2.*t24.*t135.*3.0+L2.*dq3.*t2.*t24.*t136.*6.0+L1.*dq1.*t3.*t43.*t126.*1.4e1+L1.*dq2.*t3.*t43.*t126.*2.0+L1.*dq3.*t3.*t43.*t126.*2.0+L1.*dq1.*t3.*t43.*t133.*1.1e1+L1.*dq2.*t3.*t43.*t133.*2.0+L1.*dq3.*t3.*t43.*t133+L2.*dq1.*t2.*t43.*t135-dq1.*l3.*t2.*t24.*t130.*6.0-dq2.*l3.*t2.*t24.*t130.*6.0-dq3.*l3.*t2.*t24.*t130.*6.0-dq1.*l3.*t3.*t24.*t141.*6.0-dq2.*l3.*t3.*t24.*t141.*1.8e1-dq3.*l3.*t3.*t24.*t141.*6.0-dq1.*l3.*t2.*t43.*t130-dq1.*l3.*t3.*t43.*t141.*1.8e1-dq2.*l3.*t3.*t43.*t141.*6.0-dq3.*l3.*t3.*t43.*t141.*6.0-dq1.*l3.*t2.*t127.*t130-dq1.*l3.*t3.*t127.*t141-dq2.*l3.*t3.*t127.*t141-L1.*L2.*dq1.*l3.*t24.*t131.*1.2e1-L1.*L2.*dq2.*l3.*t24.*t131.*2.4e1-L1.*L2.*dq3.*l3.*t24.*t131.*1.2e1-L1.*L2.*dq1.*l3.*t43.*t131.*1.4e1-L1.*L2.*dq2.*l3.*t43.*t131.*2.0-L1.*L2.*dq3.*l3.*t43.*t131.*3.0-L1.*L2.*dq1.*l3.*t127.*t131.*2.0-L1.*L2.*dq2.*l3.*t127.*t131+L2.*dq1.*dq2.*dq3.*t2.*t135.*2.0+L2.*dq1.*dq2.*dq3.*t2.*t136.*4.0-dq1.*dq2.*dq3.*l3.*t2.*t130.*2.0-dq1.*dq2.*dq3.*l3.*t3.*t141.*1.2e1-L1.*L2.*dq1.*dq2.*dq3.*l3.*t131.*1.4e1).*2.0,t142+t143+t144+t145+t146+t147+t148+t149+t150+t151+t159-dq1.*m3.*t2.*t3.*t24.*t125.*1.2e1-dq2.*m3.*t2.*t3.*t24.*t125.*2.4e1+dq2.*m3.*t2.*t4.*t24.*t130.*2.4e1-dq1.*m2.*t2.*t12.*t24.*t125.*1.2e1+dq3.*m3.*t2.*t4.*t24.*t130.*2.4e1-dq2.*m2.*t2.*t12.*t24.*t125.*2.4e1-dq1.*m3.*t2.*t3.*t43.*t125.*1.0e1+dq1.*m3.*t2.*t4.*t43.*t130.*1.0e1-dq1.*m2.*t2.*t12.*t43.*t125.*1.0e1+dq1.*m3.*t2.*t4.*t127.*t130.*1.0e1-L1.*L2.*dq1.*m3.*t2.*t24.*t128.*1.2e1-L1.*L2.*dq1.*m3.*t3.*t24.*t128.*1.2e1-L1.*L2.*dq2.*m3.*t2.*t24.*t128.*1.2e1-L1.*L2.*dq1.*m3.*t4.*t24.*t128.*2.4e1-L1.*L2.*dq2.*m3.*t3.*t24.*t128.*3.6e1-L1.*L2.*dq2.*m3.*t4.*t24.*t128.*7.2e1-L1.*L2.*dq3.*m3.*t4.*t24.*t128.*3.6e1+L1.*L2.*dq2.*m3.*t4.*t24.*t131.*3.6e1+L1.*L2.*dq3.*m3.*t4.*t24.*t131.*3.6e1-L1.*L2.*dq1.*m3.*t3.*t43.*t128.*3.0e1-L1.*L2.*dq1.*m3.*t4.*t43.*t128.*6.0e1-L1.*L2.*dq2.*m3.*t3.*t43.*t128.*6.0-L1.*L2.*dq2.*m3.*t4.*t43.*t128.*1.2e1-L1.*L2.*dq3.*m3.*t4.*t43.*t128.*2.0e1+L1.*L2.*dq1.*m3.*t4.*t43.*t131.*3.0e1+L1.*L2.*dq2.*m3.*t4.*t43.*t131.*6.0+L1.*L2.*dq3.*m3.*t4.*t43.*t131.*1.6e1-L1.*L2.*dq1.*m3.*t4.*t127.*t128.*1.2e1-L1.*L2.*dq2.*m3.*t4.*t127.*t128.*8.0+L1.*L2.*dq1.*m3.*t4.*t127.*t131.*2.2e1+L1.*L2.*dq2.*m3.*t4.*t127.*t131.*1.2e1-L1.*dq1.*l2.*m2.*t2.*t24.*t128.*1.2e1-L1.*dq1.*l3.*m3.*t2.*t24.*t126.*1.2e1-L1.*dq1.*l3.*m3.*t3.*t24.*t126.*2.4e1-L1.*dq2.*l2.*m2.*t2.*t24.*t128.*1.2e1-L1.*dq2.*l3.*m3.*t2.*t24.*t126.*1.2e1-L1.*dq1.*l3.*m3.*t4.*t24.*t126.*1.2e1-L1.*dq2.*l3.*m3.*t3.*t24.*t126.*7.2e1-L1.*dq3.*l3.*m3.*t2.*t24.*t126.*1.2e1-L1.*dq2.*l3.*m3.*t4.*t24.*t126.*3.6e1-L1.*dq3.*l3.*m3.*t3.*t24.*t126.*3.6e1-L1.*dq3.*l3.*m3.*t4.*t24.*t126.*3.6e1+L1.*dq2.*l3.*m3.*t3.*t24.*t133.*3.6e1-L1.*dq1.*l2.*m2.*t12.*t24.*t128.*1.2e1-L2.*dq1.*l3.*m3.*t2.*t24.*t135.*2.4e1-L1.*dq2.*l2.*m2.*t12.*t24.*t128.*3.6e1-L2.*dq2.*l3.*m3.*t2.*t24.*t135.*4.8e1-L2.*dq3.*l3.*m3.*t2.*t24.*t135.*2.4e1-L2.*dq3.*l3.*m3.*t2.*t24.*t136.*1.2e1-L2.*dq3.*l3.*m3.*t3.*t24.*t136.*1.2e1-L2.*dq3.*l3.*m3.*t4.*t24.*t136.*1.2e1-L1.*dq1.*l3.*m3.*t3.*t43.*t126.*6.0e1-L1.*dq1.*l3.*m3.*t4.*t43.*t126.*3.0e1-L1.*dq2.*l3.*m3.*t3.*t43.*t126.*1.2e1-L1.*dq2.*l3.*m3.*t4.*t43.*t126.*6.0-L1.*dq3.*l3.*m3.*t3.*t43.*t126.*1.6e1-L1.*dq3.*l3.*m3.*t4.*t43.*t126.*1.8e1+L1.*dq1.*l3.*m3.*t3.*t43.*t133.*3.0e1+L1.*dq2.*l3.*m3.*t3.*t43.*t133.*6.0-L1.*dq1.*l2.*m2.*t12.*t43.*t128.*3.0e1+L1.*dq3.*l3.*m3.*t3.*t43.*t133.*2.0-L2.*dq1.*l3.*m3.*t2.*t43.*t135.*2.0e1-L1.*dq2.*l2.*m2.*t12.*t43.*t128.*6.0-L2.*dq3.*l3.*m3.*t3.*t43.*t136.*1.2e1-L2.*dq3.*l3.*m3.*t4.*t43.*t136.*1.2e1-L1.*dq1.*l3.*m3.*t3.*t126.*t127.*8.0-L1.*dq1.*l3.*m3.*t4.*t126.*t127.*3.0e1-L1.*dq2.*l3.*m3.*t3.*t126.*t127.*4.0-L1.*dq2.*l3.*m3.*t4.*t126.*t127.*1.8e1-L1.*dq3.*l3.*m3.*t4.*t126.*t127.*6.0-L1.*dq1.*l3.*m3.*t3.*t127.*t133.*4.0-L1.*dq2.*l3.*m3.*t3.*t127.*t133.*2.0-L2.*dq1.*l3.*m3.*t2.*t127.*t135.*2.0-L2.*dq1.*l3.*m3.*t2.*t127.*t136.*4.0-L2.*dq1.*l3.*m3.*t3.*t127.*t136.*6.0-L2.*dq1.*l3.*m3.*t4.*t127.*t136.*1.8e1-L2.*dq2.*l3.*m3.*t3.*t127.*t136.*6.0-L2.*dq2.*l3.*m3.*t4.*t127.*t136.*1.8e1-L2.*dq3.*l3.*m3.*t4.*t127.*t136.*6.0+dq1.*dq2.*dq3.*m3.*t2.*t4.*t130.*2.0e1-L1.*L2.*dq1.*dq2.*dq3.*m3.*t4.*t128.*6.4e1+L1.*L2.*dq1.*dq2.*dq3.*m3.*t4.*t131.*5.6e1-L1.*dq1.*dq2.*dq3.*l3.*m3.*t3.*t126.*5.6e1-L1.*dq1.*dq2.*dq3.*l3.*m3.*t4.*t126.*6.0e1-L2.*dq1.*dq2.*dq3.*l3.*m3.*t2.*t135.*2.0e1-L2.*dq1.*dq2.*dq3.*l3.*m3.*t2.*t136.*8.0-L2.*dq1.*dq2.*dq3.*l3.*m3.*t3.*t136.*2.4e1-L2.*dq1.*dq2.*dq3.*l3.*m3.*t4.*t136.*2.4e1,t142+t146+t147+t148+t149+t151-t152-t153-t154-t155-t156-t157-t158-t159-dq2.*m3.*t2.*t3.*t24.*t125.*2.0+dq2.*m3.*t2.*t4.*t24.*t130.*2.0+dq3.*m3.*t2.*t4.*t24.*t130.*2.0-dq2.*m2.*t2.*t12.*t24.*t125.*2.0-L1.*L2.*dq2.*m3.*t3.*t24.*t128.*6.0-L1.*L2.*dq2.*m3.*t4.*t24.*t128.*1.2e1+L1.*L2.*dq2.*m3.*t4.*t24.*t131.*6.0+L1.*L2.*dq3.*m3.*t4.*t24.*t131.*1.2e1+L1.*L2.*dq1.*m3.*t4.*t43.*t131.*6.0+L1.*L2.*dq1.*m3.*t4.*t127.*t131.*8.0-L1.*dq2.*l3.*m3.*t3.*t24.*t126.*1.2e1-L1.*dq2.*l3.*m3.*t4.*t24.*t126.*6.0-L1.*dq3.*l3.*m3.*t3.*t24.*t126.*1.2e1-L1.*dq3.*l3.*m3.*t4.*t24.*t126.*6.0+L1.*dq2.*l3.*m3.*t3.*t24.*t133.*6.0-L1.*dq3.*l3.*m3.*t3.*t24.*t133.*6.0-L1.*dq2.*l2.*m2.*t12.*t24.*t128.*6.0-L2.*dq2.*l3.*m3.*t2.*t24.*t135.*4.0-L2.*dq3.*l3.*m3.*t2.*t24.*t135.*2.0-L2.*dq3.*l3.*m3.*t2.*t24.*t136.*4.0-L2.*dq3.*l3.*m3.*t3.*t24.*t136.*1.2e1-L2.*dq3.*l3.*m3.*t4.*t24.*t136.*1.2e1+L1.*dq1.*l3.*m3.*t3.*t43.*t133.*6.0-L2.*dq3.*l3.*m3.*t3.*t43.*t136.*1.2e1-L2.*dq3.*l3.*m3.*t4.*t43.*t136.*1.2e1-L1.*dq1.*l3.*m3.*t3.*t126.*t127.*4.0-L1.*dq1.*l3.*m3.*t3.*t127.*t133.*2.0-L2.*dq1.*l3.*m3.*t3.*t127.*t136.*6.0-L2.*dq1.*l3.*m3.*t4.*t127.*t136.*1.8e1-L2.*dq2.*l3.*m3.*t3.*t127.*t136.*6.0-L2.*dq2.*l3.*m3.*t4.*t127.*t136.*1.8e1-L2.*dq3.*l3.*m3.*t4.*t127.*t136.*6.0-L1.*L2.*dq1.*dq2.*dq3.*m3.*t4.*t128.*8.0+L1.*L2.*dq1.*dq2.*dq3.*m3.*t4.*t131.*1.6e1-L1.*dq1.*dq2.*dq3.*l3.*m3.*t3.*t126.*1.6e1-L2.*dq1.*dq2.*dq3.*l3.*m3.*t3.*t136.*2.4e1-L2.*dq1.*dq2.*dq3.*l3.*m3.*t4.*t136.*2.4e1,l3.*m3.*(t160+t161+t162+t163+t166+t167+t169+t171+t172-t185-t186-t190+t195+t196+t197+t198+t199-t200-t201+t202+t203+t204+t205+t206+t207-t208+t209+L1.*dq2.*t3.*t24.*t126.*1.0e1+L1.*dq3.*t3.*t24.*t126.*4.0+L1.*dq2.*t3.*t24.*t133.*1.3e1+L1.*dq3.*t3.*t24.*t133.*2.0+L2.*dq3.*t2.*t24.*t135+L2.*dq3.*t2.*t24.*t136.*2.0+L1.*dq1.*t3.*t43.*t126.*4.0+dq2.*l3.*t2.*t24.*t130+dq3.*l3.*t2.*t24.*t130-dq1.*l3.*t3.*t24.*t141.*6.0-dq2.*l3.*t3.*t24.*t141.*1.8e1-dq3.*l3.*t3.*t24.*t141.*6.0-dq1.*l3.*t3.*t43.*t141.*1.8e1-dq2.*l3.*t3.*t43.*t141.*6.0-dq3.*l3.*t3.*t43.*t141.*6.0-dq1.*l3.*t3.*t127.*t141-dq2.*l3.*t3.*t127.*t141-L1.*L2.*dq1.*l3.*t24.*t128.*6.0-L1.*L2.*dq2.*l3.*t24.*t128.*1.6e1-L1.*L2.*dq3.*l3.*t24.*t128.*6.0-L1.*L2.*dq1.*l3.*t24.*t131.*6.0-L1.*L2.*dq2.*l3.*t24.*t131.*1.0e1-L1.*L2.*dq3.*l3.*t24.*t131.*5.0-L1.*L2.*dq1.*l3.*t43.*t128.*1.0e1-L1.*L2.*dq1.*l3.*t43.*t131.*4.0-L1.*L2.*dq1.*l3.*t127.*t131+L1.*dq1.*dq2.*dq3.*t3.*t126.*4.0+L1.*dq1.*dq2.*dq3.*t3.*t133.*2.0-dq1.*dq2.*dq3.*l3.*t3.*t141.*1.2e1-L1.*L2.*dq1.*dq2.*dq3.*l3.*t128.*8.0-L1.*L2.*dq1.*dq2.*dq3.*l3.*t131.*4.0).*2.0,l3.*m3.*(t160+t161+t162+t163+t164+t165+t166+t167+t168+t169+t170+t171+t172+t173+t174+t175+t176+t177+t178+t179+t180+t181+t182+t183+t184+t187+t188+t191+t192+t193+t207+L1.*dq2.*t2.*t24.*t126.*6.0+L1.*dq2.*t3.*t24.*t126.*3.0e1+L1.*dq3.*t2.*t24.*t126.*6.0+L1.*dq2.*t4.*t24.*t126.*1.8e1+L1.*dq3.*t3.*t24.*t126.*1.2e1+L1.*dq3.*t4.*t24.*t126.*1.8e1+L1.*dq2.*t3.*t24.*t133.*1.2e1+L1.*dq3.*t3.*t24.*t133.*6.0+L2.*dq2.*t2.*t24.*t135.*1.2e1+L2.*dq3.*t2.*t24.*t135.*6.0+L2.*dq3.*t2.*t24.*t136.*1.2e1+L1.*dq1.*t3.*t43.*t126.*2.2e1+L1.*dq1.*t4.*t43.*t126.*1.5e1+L1.*dq2.*t3.*t43.*t126.*4.0+L1.*dq2.*t4.*t43.*t126.*3.0+L1.*dq3.*t3.*t43.*t126.*4.0+L1.*dq3.*t4.*t43.*t126.*9.0+L1.*dq2.*t3.*t43.*t133+L1.*dq3.*t3.*t43.*t133.*2.0+L2.*dq1.*t2.*t43.*t135.*5.0+L1.*dq1.*t4.*t126.*t127.*1.5e1+L1.*dq2.*t4.*t126.*t127.*9.0+L1.*dq3.*t4.*t126.*t127.*3.0-dq1.*l3.*t2.*t24.*t130.*6.0-dq2.*l3.*t2.*t24.*t130.*1.2e1-dq3.*l3.*t2.*t24.*t130.*1.2e1-dq1.*l3.*t3.*t24.*t141.*6.0-dq2.*l3.*t3.*t24.*t141.*1.8e1-dq3.*l3.*t3.*t24.*t141.*1.2e1-dq1.*l3.*t2.*t43.*t130.*5.0-dq1.*l3.*t3.*t43.*t141.*1.8e1-dq2.*l3.*t3.*t43.*t141.*6.0-dq3.*l3.*t3.*t43.*t141.*1.2e1-dq1.*l3.*t2.*t127.*t130.*5.0-dq1.*l3.*t3.*t127.*t141.*5.0-dq2.*l3.*t3.*t127.*t141.*5.0-L1.*L2.*dq1.*l3.*t24.*t131.*1.2e1-L1.*L2.*dq2.*l3.*t24.*t131.*3.0e1-L1.*L2.*dq3.*l3.*t24.*t131.*2.4e1-L1.*L2.*dq1.*l3.*t43.*t131.*2.2e1-L1.*L2.*dq2.*l3.*t43.*t131.*4.0-L1.*L2.*dq3.*l3.*t43.*t131.*9.0-L1.*L2.*dq1.*l3.*t127.*t131.*1.0e1-L1.*L2.*dq2.*l3.*t127.*t131.*5.0+L1.*dq1.*dq2.*dq3.*t3.*t126.*1.6e1+L1.*dq1.*dq2.*dq3.*t4.*t126.*3.0e1+L1.*dq1.*dq2.*dq3.*t3.*t133.*8.0+L2.*dq1.*dq2.*dq3.*t2.*t135.*4.0+L2.*dq1.*dq2.*dq3.*t2.*t136.*8.0-dq1.*dq2.*dq3.*l3.*t2.*t130.*1.0e1-dq1.*dq2.*dq3.*l3.*t3.*t141.*2.4e1-L1.*L2.*dq1.*dq2.*dq3.*l3.*t131.*3.4e1).*-2.0,l3.*m3.*(t160+t161+t162+t163+t164+t165+t166+t167+t168+t169+t170+t171+t172+t173+t174+t175+t176+t177-t178+t185+t186+t189+t190+t194+t197+t198+t200+t201+t202+t208+L1.*dq2.*t3.*t24.*t126.*1.4e1+L1.*dq3.*t3.*t24.*t126.*8.0+L1.*dq2.*t3.*t24.*t133.*1.1e1+L1.*dq3.*t3.*t24.*t133.*4.0+L2.*dq3.*t2.*t24.*t135.*2.0+L2.*dq3.*t2.*t24.*t136.*4.0+L1.*dq1.*t3.*t43.*t126.*8.0+L1.*dq1.*t3.*t43.*t133.*5.0-dq2.*l3.*t2.*t24.*t130-dq3.*l3.*t2.*t24.*t130-dq1.*l3.*t3.*t24.*t141.*6.0-dq2.*l3.*t3.*t24.*t141.*1.8e1-dq3.*l3.*t3.*t24.*t141.*1.2e1-dq1.*l3.*t3.*t43.*t141.*1.8e1-dq2.*l3.*t3.*t43.*t141.*6.0-dq3.*l3.*t3.*t43.*t141.*1.2e1-dq1.*l3.*t3.*t127.*t141.*5.0-dq2.*l3.*t3.*t127.*t141.*5.0-L1.*L2.*dq1.*l3.*t24.*t128.*6.0-L1.*L2.*dq2.*l3.*t24.*t128.*8.0-L1.*L2.*dq3.*l3.*t24.*t128.*6.0-L1.*L2.*dq1.*l3.*t24.*t131.*6.0-L1.*L2.*dq2.*l3.*t24.*t131.*1.4e1-L1.*L2.*dq3.*l3.*t24.*t131.*1.3e1-L1.*L2.*dq1.*l3.*t43.*t128.*2.0-L1.*L2.*dq1.*l3.*t43.*t131.*8.0-L1.*L2.*dq1.*l3.*t127.*t131.*5.0-dq1.*dq2.*dq3.*l3.*t3.*t141.*2.4e1-L1.*L2.*dq1.*dq2.*dq3.*l3.*t131.*1.4e1).*-2.0,dq3.*m3.*t4.*(-t2.*t24.*t130-t3.*t24.*t141-t3.*t43.*t141-dq1.*dq2.*t3.*t141.*2.0-L1.*L2.*t24.*t131.*2.0+L1.*l3.*t24.*t126.*3.0+L2.*l3.*t24.*t136.*3.0+L2.*l3.*t43.*t136.*3.0-L1.*L2.*dq1.*dq2.*t131.*2.0+L1.*dq1.*dq2.*l3.*t126.*3.0+L1.*dq1.*dq3.*l3.*t126.*3.0+L2.*dq1.*dq2.*l3.*t136.*6.0+L2.*dq1.*dq3.*l3.*t136.*3.0+L2.*dq2.*dq3.*l3.*t136.*3.0).*-2.0-L1.*dq1.*dq2.*m3.*t4.*(L2.*dq1.*t128.*2.0-L1.*dq1.*t130+L2.*dq2.*t128.*2.0-L2.*dq1.*t131-L2.*dq2.*t131+dq1.*l3.*t126.*3.0+dq2.*l3.*t126.*3.0+dq3.*l3.*t126.*3.0).*2.0],3,3);
end
if nargout > 2
    t210 = sparse(L1.*t30);
    t211 = sparse(L2.*t18);
    t212 = sparse(L3.*t16);
    G = sparse([1,2,3],[1,1,1],[-g.*m2.*(t210+l2.*t18)-g.*m3.*(t210+t211+t212)-g.*l1.*m1.*t30,-g.*m3.*(t211+t212)-g.*l2.*m2.*t18,-L3.*g.*m3.*t16],3,1);
end
