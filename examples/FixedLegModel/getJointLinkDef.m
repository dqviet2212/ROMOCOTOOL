function robotStr = getJointLinkDef(nJoints)
    [jointAxes, jointTypes] = getJointAxesDef();
    if (size(jointAxes, 2) == nJoints)        
        for i1 = nJoints:-1:1
            joint(i1).name = ['joint', num2str(i1)];
            joint(i1).axis = jointAxes(:, i1);
            joint(i1).type = jointTypes(i1);            
            joint(i1).origin.xyz = str2sym([['j', num2str(i1), 'x']; ['j', num2str(i1), 'y']; ['j', num2str(i1), 'z']]);
            joint(i1).origin.rpy = str2sym([['j', num2str(i1), 'r']; ['j', num2str(i1), 'p']; ['j', num2str(i1), 'a']]);
            joint(i1).parentlink = ['link', num2str(i1-1)];
            joint(i1).childlink = ['link', num2str(i1)];
            %
            link(i1).name = ['link', num2str(i1)];
            link(i1).origin.xyz = str2sym([['l', num2str(i1), 'x']; ['l', num2str(i1), 'y']; ['l', num2str(i1), 'z']]);
            link(i1).origin.rpy = str2sym([['l', num2str(i1), 'r']; ['l', num2str(i1), 'p']; ['l', num2str(i1), 'a']]);
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