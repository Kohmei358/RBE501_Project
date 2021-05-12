function Vtrans = adjoint(V,T)
% ADJOINT Calculates the 6x1 representation of a given twist "V" 
% of an arbitrary frame with respect to another frame whose position 
% and orientation are given by homogenous transformation matrix 'T'.
% 
%
% Inputs: V - Twist of a arbitrary screw axis 
%         T - Homogenous Transformation Matrix
%
% Output: VTrans - Representation of twist 'V' with respect to
%                  transformation Matrix 'T'
%
% RBE 501 - Robot Dynamics - Spring 2021
% Worcester Polytechnic Institute 
%
% Student: Sumanth Varma Pericherla <spericherla@wpi.edu>

    % Rotation and orientation are removes from the transformation matrix
    R = T(1:3,1:3);
    P = T(1:3,4);
    
    % Calculating a skew-symmetric matrix with the given position
    skewP = [0  -P(3) P(2); P(3) 0 -P(1); -P(2) P(1) 0];
    z = zeros(3);
    
    % Calculating the adjoint transformation of the given homogenous matrix
    adjT = [R z; skewP*R R];
    
    % Multiplying the Adjoint Matrix with the given twist for calculating
    % the transformation
    Vtrans = adjT*V;
    
end
