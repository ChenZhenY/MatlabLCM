function [matrix] = rotationMatrix(angle,axis)
%ROTATIONMATRIX rotate angle along certain axis
%   default: angle in radians, direction follow right hand rule (ccw?)
matrix = "error";
if axis == 'x'
    matrix = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];
end
if axis == 'y'
    matrix = [cos(angle), 0, sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
end
if axis == 'z'
    matrix = [cos(angle), -sin(angle), 0; sin(angle), cos(angle), 0; 0 0 1];
end
end

