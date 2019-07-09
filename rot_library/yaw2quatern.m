function q = yaw2quatern(psi)
%   Converts ZYX Euler angle orientation to a quaternion where phi is
%   a rotation around X, theta around Y and psi around Z.
q = zeros(1,4);
q(1) = cos(psi/2);
q(4) = sin(psi/2);
end