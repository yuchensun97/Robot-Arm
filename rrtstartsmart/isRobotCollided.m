function flag = isRobotCollided(q,map);
% This function detect whether a node is in the obstacle
% Input:
%   map -- a struct containing the boundary and obstacle
%   q -- a node's position
% Output:
%   flag -- a boolean that shows whether a node is in the obstacle. If flag
%   = 0, the node is not in the obstacle. If flag = 1, the node is in the
%   obstacle
%% Algorithm starts here
[numObstacle,~] = size(map.obstacle);
obstacle = map.obstacle;
for i = 1:numObstacle
    if q(1)> obstacle(i,1) && q(1) < obstacle(i,1)+obstacle(i,3) && q(2) > obstacle(i,2) && q(2) < obstacle(i,2)+obstacle(i,4)
        flag = 1;
    else
        flag = 0;
    end
end