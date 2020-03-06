function flag = detectCollision(q1,q2,obstacles)
%   This function returns a flag that tells whether to nodes are connected
%   Input:
%   q1: a node, 1X2 Vector
%   q2: a node, 1X2 Vector
%   Output:
%   flag: a boolean whether two nodes are connected. If flag = 1, nodes are
%   not connected. If flag = 0, nodes are connected
%% Algorithm starts here
% obstacle = map.obstalce;
% clc
% clear
% q1 = [150,150];
% q2 = [45,88];
[numObstacle,~]=size(obstacle);
flag_1 = 0;
flag_2 = 0;
flag = 0;
for i = 1:numObstacle
    if q1(1)> obstacle(i,1) && q1(1) < obstacle(i,1)+obstacle(i,3) && q1(2) > obstacle(i,2) && q1(2) < obstacle(i,2)+obstacle(i,4)
        flag_1 = 1;
    end
    if q2(1)> obstacle(i,1) && q2(1) < obstacle(i,1)+obstacle(i,3) && q2(2) > obstacle(i,2) && q2(2) < obstacle(i,2)+obstacle(i,4)
        flag_2 = 1;
    end
   if flag_1 == 0 && flag_2 == 0
       flag = 0;
   else
       flag = 1;
   end
   if flag ~=0
       break;
   end
end
end