function rotM = Rpy2Mat(rpy)
r = rpy(1);     % x-axis
p = rpy(2);     % y-axis
y = rpy(3);     % z-axis
rotM = [cos(p)*cos(y), -cos(p)*sin(y), sin(p);...
        sin(r)*sin(p)*cos(y) + cos(r)*sin(y), -sin(r)*sin(p)*sin(y) + cos(r)*cos(y), -sin(r)*cos(p);...
        -cos(r)*sin(p)*cos(y) + sin(r)*sin(y), cos(r)*sin(p)*sin(y) + sin(r)*cos(y), cos(r)*cos(p)];
end