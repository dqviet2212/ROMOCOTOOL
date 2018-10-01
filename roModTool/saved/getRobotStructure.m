function robot = getRobotStructure(robot)
    %% Base
    robot.base.name = 'base';
    robot.base.origin.xyz = str2sym(['px'; 'py'; 'pz']);
    robot.base.origin.rpy = str2sym(['ro'; 'po'; 'yo']);
    robot.base.origin.dxyz = str2sym(['dpx'; 'dpy'; 'dpz']);
    robot.base.origin.drpy = str2sym(['dro'; 'dpo'; 'dyo']);                           
    robot.base.mass = sym('mB');
    robot.base.inertia(1, 1) = sym('ixxB');
    robot.base.inertia(1, 2) = sym('ixyB');
    robot.base.inertia(1, 3) = sym('ixzB');
    robot.base.inertia(2, 1) = sym('ixyB');
    robot.base.inertia(2, 2) = sym('iyyB');
    robot.base.inertia(2, 3) = sym('iyzB');
    robot.base.inertia(3, 1) = sym('ixzB');
    robot.base.inertia(3, 2) = sym('iyzB');
    robot.base.inertia(3, 3) = sym('izzB');
    robot.base.symJointVar = [robot.base.origin.xyz;...
                              robot.base.origin.rpy];
    robot.base.symdJointVar = [robot.base.origin.dxyz;...
                               robot.base.origin.drpy];
    robot.base.symPhysicalParams = [robot.base.mass;...
                                    robot.base.inertia(1, 1);...
                                    robot.base.inertia(1, 2);...
                                    robot.base.inertia(1, 3);...
                                    robot.base.inertia(2, 2);...
                                    robot.base.inertia(2, 3);...
                                    robot.base.inertia(3, 3)];
    
    if strcmp(robot.base.type, 'fixed')        
        robot.base.origin.xyz = sym(zeros(3, 1));
        robot.base.origin.rpy = sym(zeros(3, 1));
        robot.base.origin.dxyz = sym(zeros(3, 1));        
        robot.base.origin.drpy = sym(zeros(3, 1));
        robot.base.mass = sym(zeros(1));
        robot.base.inertia = sym(zeros(3));
    end
    
    %% Kinematic chain                                    
    nKinematicChains = size(robot.kinematicChains, 2);    
    for i1 = nKinematicChains:-1:1
        nJoints = size(robot.kinematicChains(i1).joint, 2);
        for i2 = nJoints:-1:1
            robot.kinematicChains(i1).joint(i2).name = ['joint', num2str(i1), num2str(i2)];
            robot.kinematicChains(i1).joint(i2).origin.xyz = str2sym([['j', num2str(i1), num2str(i2), 'xp']; ['j', num2str(i1), num2str(i2), 'yp']; ['j', num2str(i1), num2str(i2), 'zp']]);
            robot.kinematicChains(i1).joint(i2).origin.rpy = str2sym([['j', num2str(i1), num2str(i2), 'ro']; ['j', num2str(i1), num2str(i2), 'po']; ['j', num2str(i1), num2str(i2), 'yo']]);
            robot.kinematicChains(i1).joint(i2).parentlink = ['link', mat2str([i1, i2-1])];
            robot.kinematicChains(i1).joint(i2).childlink = ['link', num2str(i1), num2str(i2)];
            robot.kinematicChains(i1).joint(i2).symPhysicalParams = [robot.kinematicChains(i1).joint(i2).origin.xyz;...
                                                                     robot.kinematicChains(i1).joint(i2).origin.rpy];
            %
            robot.kinematicChains(i1).link(i2).name = ['link', num2str(i1), num2str(i2)];
            robot.kinematicChains(i1).link(i2).origin.xyz = str2sym([['l', num2str(i1), num2str(i2), 'xp']; ['l', num2str(i1), num2str(i2), 'yp']; ['l', num2str(i1), num2str(i2), 'zp']]);
            robot.kinematicChains(i1).link(i2).origin.rpy = str2sym([['l', num2str(i1), num2str(i2), 'ro']; ['l', num2str(i1), num2str(i2), 'po']; ['l', num2str(i1), num2str(i2), 'yo']]);
            robot.kinematicChains(i1).link(i2).mass = str2sym(['m', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(1, 1) = str2sym(['ixx', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(1, 2) = str2sym(['ixy', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(1, 3) = str2sym(['ixz', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(2, 1) = str2sym(['ixy', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(2, 2) = str2sym(['iyy', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(2, 3) = str2sym(['iyz', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(3, 1) = str2sym(['ixz', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(3, 2) = str2sym(['iyz', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).inertia(3, 3) = str2sym(['izz', num2str(i1), num2str(i2)]);
            robot.kinematicChains(i1).link(i2).symPhysicalParams = [robot.kinematicChains(i1).link(i2).origin.xyz;...
                                                                    robot.kinematicChains(i1).link(i2).origin.rpy;...
                                                                    robot.kinematicChains(i1).link(i2).mass;...
                                                                    robot.kinematicChains(i1).link(i2).inertia(1, 1);...
                                                                    robot.kinematicChains(i1).link(i2).inertia(1, 2);...
                                                                    robot.kinematicChains(i1).link(i2).inertia(1, 3);...
                                                                    robot.kinematicChains(i1).link(i2).inertia(2, 2);...
                                                                    robot.kinematicChains(i1).link(i2).inertia(2, 3);...
                                                                    robot.kinematicChains(i1).link(i2).inertia(3, 3)];
        end
    end
end