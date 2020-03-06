function [startPos goalPos]= endPos(qstart, qgoal)
% this function will get the position of end effector of q start and q goal
% input:
%       qstart: The start configuration space, 1X6 vector
%       qgoal: The goal configuration space , 1X6 vector
% output:
%       startPos: The position of end effector at start C space, 1X3 vector
%       goalPos: The position of end effector at goal C space, 1X3 vector
%% Code starts here
addpath('utils')
addpath('maps')
[jointPos_start, T_start]=calculateFK_sol(qstart);
startPos = (T_start(1:3,4))';
[jointPos_goal, T_goal]=calculateFK_sol(qgoal);
goalPos = (T_goal(1:3,4))';
end
%% Code ends