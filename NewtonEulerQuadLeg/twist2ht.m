function T = twist2ht(S,theta)
% Your code here
    omega = S(1:3);
    skewomega = [0  -omega(3) omega(2); omega(3) 0 -omega(1); -omega(2) omega(1) 0];
    R = axisangle2rot(omega,theta);
    v = S(4:6);
    e_s_theta = [R ((eye(3)*theta) + (1-cos(theta))*skewomega+(theta - sin(theta))*(skewomega^2))*v; 0 0 0 1];
    T = e_s_theta;
end