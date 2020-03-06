function [isCollided] = isRobotCollided(q, map, robot)
% ISROBOTCOLLIDED Detect if a configuration of the Lynx is in collision
%   with any obstacles on the map.
%
% INPUTS:
%   q   - a 1x6 configuration of the robot
%   map - a map strucutre containing axis-aligned-boundary-box obstacles
%   robot - a structure containing the robot dimensions and joint limits
%
% OUTPUTS:
%   isCollided - a boolean flag: 1 if the robot is in collision, 0 if not.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[jointPos, T] = calculateFK_sol(q);
[r,c] =size(map.obstacles);
for i = 1:5
    line1 = jointPos(i,1:3);
    line2 = jointPos(i+1,1:3);
    for j = 1:r
        iscollid = detectCollision(line1,line2,map.obstacles(j,:));
        if iscollid ~= 0
            break;
        end
    end
    if iscollid ~=0
        break;
    end
end
if iscollid ==0
    isCollided = 0;
else
    isCollided = 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end