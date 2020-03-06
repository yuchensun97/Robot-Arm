function [qNext, isDone] = potentialFieldStep_22(qCurr, map, robot)
% POTENTIALFIELDSTEP_GROUPNO Calculates a single step in a potential field
%   planner based on the virtual forces exerted by all of the elements in
%   map. This function will be called over and over until isDone is set.
%   Use persistent variables if you need historical information. CHANGE 
%   GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   qCurr - 1x6 vector representing the current configuration of the robot.
%   map   - a map struct containing the boundaries of the map, any
%           obstacles, the start position, and the goal position.
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   qNext  - 1x6 vector representing the next configuration of the robot
%            after it takes a single step along the potential field.
%   isDone - a boolean flag signifying termination of the potential field
%            algorithm. 
%
%               isDone == 1 -> Terminate the planner. We have either
%                              reached the goal or are stuck with no 
%                              way out.
%               isDone == 0 -> Keep going.

%% for debugging, comment this part when run the function
% addpath('utils')
% clc
% clear
% close all
% robot = load('robot.mat');
% map = loadmap('map4.txt');
% qCurr = [0.4676    0.2612    0.5750    0.0960   -0.0089   -0.0000];
% goal = [0.6000    1.2000         0         0         0         0];
%% code start here
global goal
qNext = zeros(1,6);
qNext = qCurr;
qNext(1) = qNext(1)+.01;
isDone = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% initialization
obstacle = map.obstacles;
[number_obstacle,~] = size(obstacle);
rou0 = 100; % distance of influence of the obstacles, in mm
eta = 5000; % get the repulsive strength
zeta = [3 3 3 3 3 3 5]; % get the attractive field strength
[goal_pos,~] = calculateFK_sol(goal); % get each joint's goal position
[curr_pos,~] = calculateFK_sol(qCurr); % get each joint's current position
dist_att = 100; % when joint is far away from goal, use conic, else use parabolic
alpha = 0.005; % step size
sumFrep = zeros(7,3);
%% get the shortest distance and the corresponding point
for i=1:number_obstacle
    [rou(:,i) b{1,i}]=distPointToBox(curr_pos,obstacle(i,:));
end
%% calculate Fatt and Frep
for i=1:7
    % calculate Fatt
    dist = norm(goal_pos(i,:)-curr_pos(i,:)); % calculate each joint's distance from current position to goal position
    if dist > dist_att
        Fatt(i,:) = -(curr_pos(i,:)-goal_pos(i,:))/norm(curr_pos(i,:)-goal_pos(i,:));
    else
        Fatt(i,:) = -zeta(i)*(curr_pos(i,:)-goal_pos(i,:));
    end
    % calculate Frep
    for j = 1:number_obstacle
        if rou(i,j)>rou0
            Frep{i,j} = [0 0 0];
        else if rou(i,j)<=rou0
                Frep{i,j} = eta*(1/rou(i,j)-1/rou0)*1/((rou(i,j))^2)*(curr_pos(i,:)-b{1,j}(i,:))/norm(curr_pos(i,:)-b{1,j}(i,:));
            end
        end
%        F(i,:) = [Fatt(i,:)+Frep{i,j}(1,:) 0 0 0];
        sumFrep(i,:)=Frep{i,j}+sumFrep(i,:);
    end
    F(i,:)=[Fatt(i,:)+sumFrep(i,:) 0 0 0];
end
% for i = 1:7
%     for j = 1:number_obstacle
%         F(i,:)=[Fatt(i,:)+Frep{i,j} 0 0 0];
%     end
% end
%% calculate each joint's torque
for i = 1:7
    J{1,i}=calcJacobian_22(qCurr,i,robot);
end
tau7 = (J{1,7})'*(F(7,:))';
tau6 = [(J{1,6})'*(F(6,:))';0];
tau5 = [(J{1,5})'*(F(5,:))';0;0];
tau4 = [(J{1,4})'*(F(4,:))';0;0;0];
tau3 = [(J{1,3})'*(F(3,:))';0;0;0;0];
tau2 = [(J{1,2})'*(F(2,:))';0;0;0;0;0];

tau = tau2+tau3+tau4+tau5+tau6+tau7;
qNext = qCurr+alpha*tau'/norm(tau);

if norm(qNext-goal)<0.1;
    isDone = 1;
else
    isDone = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end