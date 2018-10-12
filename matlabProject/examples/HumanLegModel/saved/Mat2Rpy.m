function rpy = Mat2Rpy(rotM)
    r = atan2(-rotM(2, 3), rotM(3, 3));
    p = asin(rotM(1, 3));               % -pi/2 <= p <= +pi/2
    y = atan2( -rotM(1, 2), rotM(1, 1));    
    rpy = [r; p; y];
end