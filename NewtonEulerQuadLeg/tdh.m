function T = tdh(theta, d, a, alpha)
    RotZ = [ cos(theta), -sin(theta), 0, 0; sin(theta), cos(theta), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    TraZ = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, d;, 0, 0, 0, 1];
    TraX = [1, 0, 0, a; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
    RotX = [ 1, 0, 0, 0; 0, cos(alpha), -sin(alpha), 0; 0, sin(alpha), cos(alpha), 0; 0, 0, 0, 1];
    
    
    T = RotZ * TraZ * TraX * RotX;
end
