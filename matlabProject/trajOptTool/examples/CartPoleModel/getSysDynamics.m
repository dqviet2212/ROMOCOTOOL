function dotx = getSysDynamics(x, u, sysParams)
    m1  = sysParams.m1;
    m2  = sysParams.m2;
    l   = sysParams.l;
    g   = sysParams.g;
    %
    x2 = x(2, :);
    x3 = x(3, :);
    x4 = x(4, :);
    dotx1 = x3;
    dotx2 = x4;
    dotx3 = (m2*l*sin(x2).*(x4.^2) + m2*g*cos(x2).*sin(x2) + u)./(m1 + m2*(1 - (cos(x2)).^2));
    dotx4 = -(m2*l*cos(x2).*sin(x2).*(x4.^2) + (m1 + m2)*g*sin(x2) + (cos(x2)).*u)./(m1*l + m2*l*(1 - (cos(x2)).^2));
    dotx = [dotx1; dotx2; dotx3; dotx4];    
end