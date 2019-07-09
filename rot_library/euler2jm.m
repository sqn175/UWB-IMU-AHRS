 function jm = euler2jm(phi, theta, psi)
% Input: ZYX euler angles where phi is
%        a rotation around X, theta around Y and psi around Z
% Output: The transition matrix from Euler rates to the 
% body-frame angular velocity vector.
% By: ShiQin
    sx = sin(phi); 
    cx = cos(phi);
    sy = sin(theta);
    cy = cos(theta);
    % Z-Y-X euler angle sequence 
    jm = [1 0 -sy;
          0 cx sx*cy;
          0 -sx cx*cy];
end