function [path] = rrt(map, start, goal)
% RRT Find the shortest path from start to goal.
%   PATH = rrt(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The 
%   first row is start and the last row is goal. If no path is found, PATH 
%   is a 0x6 matrix. 
%
% INPUTS:
%   map     - the map object to plan in
%   start   - 1x6 vector of the starting configuration
%   goal:   - 1x6 vector of the goal configuration


%% Prep Code

tic
close all
addpath('maps')
addpath('utils')
ro=size(map.obstacles,1);
robot = load('robot.mat');
[start_pos goal_pos]= endPos(start,goal);
T_start = start; %store configuration space in T_start
T_spos = start_pos; %store random points' position in T_spos
T_goal = goal; % store configuration space in T_goal
T_gpos = goal_pos; % store random points' position in T_gpos
step = 1; %store the number of step
row_T_s = 1;% store the rows in T_start
row_T_g = 1; %store the rows in T_goal
closest_row_s = [1]; %store the row of each closest node in Tstart
closest_row_g = [1]; %store the row of each closest node in Tgoal
max_distance = 1; %set the maximun distance to 1, in configuration space 
path_flag = 0;
q(4) = 0; q(5) = 0; q(6) = 0;
checkcollision = [];
%lynxStart;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for iter = 1:10000
    q(1) = -1.4+2.8*rand; 
    q(2) = -1.2+2.6*rand;
    q(3) = -1.8+3.5*rand;
    iscollision_1 = isRobotCollided(q,map,robot);
    %if the robot is not in collision
    if iscollision_1 == 0 %if q is in Qfree
        step = step+1;
        [q_a,min_distance_s,min_pos_s]=closestNode(T_start,q);
        [q_a_w,~] = calculateFK_sol(q_a);
        [q_w,~]=calculateFK_sol(q);
        for n = 1:50
                checkcollision(1) = n/50*abs(q_a(1)-q(1))+min(q_a(1),q(1));
                checkcollision(2) = n/50*abs(q_a(2)-q(2))+min(q_a(1),q(1));
                checkcollision(3) = n/50*abs(q_a(3)-q(3))+min(q_a(1),q(1));
                checkcollision(4) = n/50*abs(q_a(4)-q(4))+min(q_a(1),q(1));
                checkcollision(5) = n/50*abs(q_a(5)-q(5))+min(q_a(1),q(1));
                checkcollision(6) = n/50*abs(q_a(6)-q(6))+min(q_a(1),q(1));
                iscollision_2 = isRobotCollided(checkcollision,map,robot);
            if iscollision_2 ~=0
                break
            end
        end
        
        [q_b,min_distance_g,min_pos_g]=closestNode(T_goal,q);
        [q_b_w,~] = calculateFK_sol(q_b);
        for n = 1:50
                checkcollision(1) = n/50*abs(q_b(1)-q(1))+min(q_b(1),q(1));
                checkcollision(2) = n/50*abs(q_b(2)-q(2))+min(q_b(1),q(1));
                checkcollision(3) = n/50*abs(q_b(3)-q(3))+min(q_b(1),q(1));
                checkcollision(4) = n/50*abs(q_b(4)-q(4))+min(q_b(1),q(1));
                checkcollision(5) = n/50*abs(q_b(5)-q(5))+min(q_b(1),q(1));
                checkcollision(6) = n/50*abs(q_b(6)-q(6))+min(q_b(1),q(1));
                iscollision_3 = isRobotCollided(checkcollision,map,robot);
            if iscollision_3 ~=0
                break
            end
        end
        
       if (iscollision_2 ~= 0) && (iscollision_3 ~=0 )
           continue
       end
       
       if iscollision_2 ==0 && min_distance_s<max_distance
           row_T_s = row_T_s +1;
           T_start(row_T_s,:) = q;
           T_spos(row_T_s,:) = q_w(6,:);
           closest_row_s(row_T_s,:)=min_pos_s;
           start_flag = 1;
       else
           start_flag = 0;
       end
       if iscollision_3 ==0 && min_distance_g<max_distance
           row_T_g = row_T_g +1;
           T_goal(row_T_g,:)=q;
           T_gpos(row_T_g,:)=q_w(6,:);
           closest_row_g(row_T_g,:)=min_pos_g;
           goal_flag = 1;
       else
           goal_flag = 0;
       end
        %if q is connected to Tstart and Tgoal, path is found and break the
        if (start_flag ==1) && (goal_flag ==1)
            disp('path found!')
            path_flag = 1; %a boolean, if there is a path, set the flag to 1
            break;
        end
    end
end
%% find the node along the path
if path_flag == 0
    disp('no path found!Please try again!')
    path = [];
else 
    r_s = size(T_start,1);
    r_g = size(T_goal,1);
    parent_s_q(1,:) = T_start(r_s,:);
    parent_g_q(1,:) = T_goal(r_g,:);
    order_s = 1;
    order_g = 1;
    T_start_order = T_start;
    T_start_order(r_s,:)=[];
    T_goal_order = T_goal;
    T_goal_order(r_g,:)=[];
    while(1)
        [q_parent_s, distance_s, smin_pos] = closestNode(T_start_order,parent_s_q(order_s,:));
        T_start_order(smin_pos,:)=[];
        order_s = order_s+1;
        parent_s_q(order_s,:) = q_parent_s;
        if smin_pos ==1
            break;
        end
    end
    while(1)
        [q_parent_g,distance_g,gmin_pos] = closestNode(T_goal_order, parent_g_q(order_g,:));
        T_goal_order(gmin_pos,:)=[];
        order_g = order_g+1;
        parent_g_q(order_g,:) = q_parent_g;
        if gmin_pos == 1
            break;
        end
    end
end
%% Find the path in C-Space
if path_flag == 1
r_ps = size(parent_s_q,1);
r_pg = size(parent_g_q,1);
parent_s_right = flip(parent_s_q,1);
for n=1:r_ps
    path(n,:)=parent_s_right(n,:);
end
path(r_ps+1:r_ps+r_pg-1,:)=parent_g_q(2:r_pg,:);
%% Find the path in Workspace
for m=1:r_ps+r_pg-1
    [PathPos,PathT]=calculateFK_sol(path(m,:));
    path_W(m,:)=PathPos(6,:);
end
%% plot the path
plotmap(map);
scatter3(T_spos(:,1),T_spos(:,2),T_spos(:,3),'g','filled');
scatter3(T_gpos(:,1),T_gpos(:,2),T_gpos(:,3),'r','filled');
plot3(path_W(:,1),path_W(:,2),path_W(:,3),'LineWidth',2);
end
toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end