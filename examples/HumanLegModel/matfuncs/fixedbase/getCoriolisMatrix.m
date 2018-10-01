function coriolisMatrix = getCoriolisMatrix(in1,in2)
%GETCORIOLISMATRIX
%    CORIOLISMATRIX = GETCORIOLISMATRIX(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    01-Oct-2018 18:24:06

%======================================================================%
% Description: This function computes the Coriolis matrix C of robot dynamics
% coriolisMatrix = getCoriolisMatrix(physicalParams, stateVariable)
% @Inputs:
% physicalParams: Physical parameters of the dynamical system - column vector
% stateVariable: State variable of the dynamical system - column vector
% @Output:
% coriolisMatrix: Coriolis matrix of robot dynamics
% Version: 1.0
% Author: Quoc-Viet Dang
%======================================================================%
dq11 = in2(3,:);
dq21 = in2(4,:);
j11po = in1(6,:);
j11ro = in1(5,:);
j11yo = in1(7,:);
j21po = in1(12,:);
j21ro = in1(11,:);
j21yo = in1(13,:);
l11xp = in1(14,:);
l11zp = in1(16,:);
l21xp = in1(27,:);
l21zp = in1(29,:);
m11 = in1(20,:);
m21 = in1(33,:);
q11 = in2(1,:);
q21 = in2(2,:);
t2 = conj(j11ro);
t3 = conj(j11yo);
t4 = conj(j11po);
t5 = cos(t2);
t6 = conj(q11);
t7 = sin(t6);
t8 = sin(t2);
t9 = sin(t3);
t10 = t8.*t9;
t11 = cos(t3);
t12 = sin(t4);
t27 = t5.*t11.*t12;
t13 = t10-t27;
t14 = cos(t4);
t15 = cos(t6);
t16 = cos(j11ro);
t17 = cos(q11);
t18 = sin(j11ro);
t19 = sin(j11yo);
t20 = t18.*t19;
t21 = cos(j11yo);
t22 = sin(j11po);
t34 = t16.*t21.*t22;
t23 = t20-t34;
t24 = cos(j11po);
t25 = sin(q11);
t26 = conj(l11xp);
t28 = t7.*t13;
t29 = t5.*t14.*t15;
t30 = t28+t29;
t31 = conj(l11zp);
t32 = t13.*t15;
t33 = t32-t5.*t7.*t14;
t35 = t17.*t23;
t36 = t35-t16.*t24.*t25;
t37 = t23.*t25;
t38 = t16.*t17.*t24;
t39 = t37+t38;
t40 = t7.*t12;
t41 = t40-t11.*t14.*t15;
t42 = t12.*t15;
t43 = t7.*t11.*t14;
t44 = t42+t43;
t45 = t17.*t22;
t46 = t21.*t24.*t25;
t47 = t45+t46;
t48 = t22.*t25;
t49 = t48-t17.*t21.*t24;
t50 = t5.*t9;
t51 = t8.*t11.*t12;
t52 = t50+t51;
t53 = t16.*t19;
t54 = t18.*t21.*t22;
t55 = t53+t54;
t56 = t7.*t52;
t57 = t56-t8.*t14.*t15;
t58 = t15.*t52;
t59 = t7.*t8.*t14;
t60 = t58+t59;
t61 = t17.*t55;
t62 = t18.*t24.*t25;
t63 = t61+t62;
t64 = t25.*t55;
t65 = t64-t17.*t18.*t24;
t66 = conj(j21ro);
t67 = conj(j21yo);
t68 = conj(j21po);
t69 = cos(t66);
t70 = conj(q21);
t71 = sin(t70);
t72 = sin(t66);
t73 = sin(t67);
t74 = t72.*t73;
t75 = cos(t67);
t76 = sin(t68);
t91 = t69.*t75.*t76;
t77 = t74-t91;
t78 = cos(t68);
t79 = cos(t70);
t80 = cos(j21ro);
t81 = cos(q21);
t82 = sin(j21ro);
t83 = sin(j21yo);
t84 = t82.*t83;
t85 = cos(j21yo);
t86 = sin(j21po);
t98 = t80.*t85.*t86;
t87 = t84-t98;
t88 = cos(j21po);
t89 = sin(q21);
t90 = conj(l21xp);
t92 = t71.*t77;
t93 = t69.*t78.*t79;
t94 = t92+t93;
t95 = conj(l21zp);
t96 = t77.*t79;
t97 = t96-t69.*t71.*t78;
t99 = t81.*t87;
t100 = t99-t80.*t88.*t89;
t101 = t87.*t89;
t102 = t80.*t81.*t88;
t103 = t101+t102;
t104 = t71.*t76;
t105 = t104-t75.*t78.*t79;
t106 = t76.*t79;
t107 = t71.*t75.*t78;
t108 = t106+t107;
t109 = t81.*t86;
t110 = t85.*t88.*t89;
t111 = t109+t110;
t112 = t86.*t89;
t113 = t112-t81.*t85.*t88;
t114 = t69.*t73;
t115 = t72.*t75.*t76;
t116 = t114+t115;
t117 = t80.*t83;
t118 = t82.*t85.*t86;
t119 = t117+t118;
t120 = t71.*t116;
t121 = t120-t72.*t78.*t79;
t122 = t79.*t116;
t123 = t71.*t72.*t78;
t124 = t122+t123;
t125 = t81.*t119;
t126 = t82.*t88.*t89;
t127 = t125+t126;
t128 = t89.*t119;
t129 = t128-t81.*t82.*t88;
coriolisMatrix = reshape([dq11.*m11.*((l11xp.*t36+l11zp.*t39).*(t26.*t30-t31.*t33).*2.0-(l11zp.*t36-l11xp.*t39).*(t26.*t33+t30.*t31).*2.0-(l11xp.*t47+l11zp.*t49).*(t26.*t41-t31.*t44).*2.0-(l11xp.*t49-l11zp.*t47).*(t26.*t44+t31.*t41).*2.0+(l11xp.*t63+l11zp.*t65).*(t26.*t57-t31.*t60).*2.0+(l11xp.*t65-l11zp.*t63).*(t26.*t60+t31.*t57).*2.0).*(1.0./4.0),0.0,0.0,dq21.*m21.*((l21xp.*t100+l21zp.*t103).*(t90.*t94-t95.*t97).*2.0-(l21zp.*t100-l21xp.*t103).*(t90.*t97+t94.*t95).*2.0-(l21xp.*t111+l21zp.*t113).*(t90.*t105-t95.*t108).*2.0-(l21xp.*t113-l21zp.*t111).*(t90.*t108+t95.*t105).*2.0+(l21xp.*t127+l21zp.*t129).*(t90.*t121-t95.*t124).*2.0+(l21xp.*t129-l21zp.*t127).*(t90.*t124+t95.*t121).*2.0).*(1.0./4.0)],[2,2]);