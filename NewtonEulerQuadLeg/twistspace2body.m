function V_b = twistspace2body(V_s,T)
    % your code here
    R = T(1:3,1:3);
    P = T(1:3,4);
    skewP = [0  -P(3) P(2); P(3) 0 -P(1); -P(2) P(1) 0];
    z = zeros(3);
    adjT = [R z; skewP*R R];
    
    V_b = inv(adjT)*V_s;
end