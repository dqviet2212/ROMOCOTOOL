function robotStr = getRobotStructure(L1, L2, L3, l1, l2, l3, m1, m2, m3)
    %% Base    
    base.name = 'base';
    base.origin.xyz = sym('pB', [3, 1]);
    base.origin.rpy = sym('rB', [3, 1]);
    base.mass = sym('mB');
    base.inertia = sym('iB', [3, 3]);
%     base.name = 'base';
%     base.origin.xyz = [0; 0; 0];
%     base.origin.rpy = [0; 0; 0];
%     base.mass = 0;
%     base.inertia = zeros(3);
    
    %% Joints
    % Joint1
    joint(1).name = 'joint1';
    joint(1).type = 'revolute';
    joint(1).axis = [0; 1; 0];
    joint(1).origin.xyz = [0; 0; 0];
    joint(1).origin.rpy = [0; 0; 0];
    joint(1).parentlink = 'base';
    joint(1).childlink = 'link1';
    % Joint2
    joint(2).name = 'joint2';
    joint(2).type = 'revolute';
    joint(2).axis = [0; 1; 0];
    joint(2).origin.xyz = [0; 0; L1];
    joint(2).origin.rpy = [0; 0; 0];
    joint(2).parentlink = 'link1';
    joint(2).childlink = 'link2';
    % Joint3
    joint(3).name = 'joint3';
    joint(3).type = 'revolute';
    joint(3).axis = [0; 1; 0];
    joint(3).origin.xyz = [0; 0; L2];
    joint(3).origin.rpy = [0; 0; 0];
    joint(3).parentlink = 'link2';
    joint(3).childlink = 'link3';
    
    %% Links
    % Link1
    link(1).name = 'link1';
    link(1).origin.xyz = [0; 0; l1];
    link(1).origin.rpy = [0; 0; 0];
    link(1).mass = m1;
    link(1).inertia = [(1/12)*m1*L1^2, 0,                   0;...
                       0,              (1/12)*m1*L1^2,      0;...
                       0,              0,                   0];
    % Link2
    link(2).name = 'link2';
    link(2).origin.xyz = [0; 0; l2];
    link(2).origin.rpy = [0; 0; 0];
    link(2).mass = m2;
    link(2).inertia = [(1/12)*m2*L2^2, 0,                   0;...
                       0,              (1/12)*m2*L2^2,      0;...
                       0,              0,                   0];
                   
    % Link3
    link(3).name = 'link3';
    link(3).origin.xyz = [l3; 0; 0];
    link(3).origin.rpy = [0; 0; 0];
    link(3).mass = m3;
    link(3).inertia = [0,              0,                   0;...
                       0,              (1/12)*m1*L3^2,      0;...
                       0,              0,                   (1/12)*m1*L3^2];
    
    %% Robot structure
    robotStr.base = base;
    robotStr.joint = joint;
    robotStr.link = link;
end

