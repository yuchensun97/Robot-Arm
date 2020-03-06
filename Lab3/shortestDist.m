function [dist, b] = shortestDist(point_out,center,block)
%This function calculate the point projecting on a particular plane
%Input:
%   point_out: a 1X3 vector representing a line out of the plane
%   center: a 1X3 vector representing the center of the block
%   block: a 1X6 vector representing the obstacle
%Output:
%   dist: a constant, representing the shortest distance between oi and the
%   obstacles
%   b: a 1X3 vector, representing the point on the obstacle boundary cloest
%   to oi
%% Algorithm Start Here
 diag = block(4:6)-block(1:3); % get the diagnol of the block
 line = abs(point_out)-center; % get the vector from center to the outter point
 positive = 0;
% count the number of element that is positive
 for i = 1:3
     if line(i)>0
         positive = positive+1;
     end
 end
% ananlyze different cases
if positive == 0 % if the positive element is 0, the point is in the block
    dist = 0;
    b = [];
else if positive ==1 % if the positive element is 1, the point face the block
     dist = norm(line);
     b = 
end
