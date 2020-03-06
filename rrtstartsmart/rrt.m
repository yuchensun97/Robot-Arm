% rrt algorithm in 2D with collision avoidance
%
% Aurthor: Shallchih Shih, Yuchen Sun, Yifan Yuan
%
% RRT finds the shortest path from start to goal
%% Algorithm starts here
%clc
tic

for run = 1:1
    
clear
close all
%% Initialization
% initialize the map, maybe later write as a subfunction
map.x_max = 1000;
map.y_max = 1000;

totallength = 0;

%map 1
map.obstacle(1,:) = [0, 50, 25, 1000];
map.obstacle(2,:) = [25, 975, 675, 1000];
map.obstacle(3,:) = [325, 0, 350, 775];
map.obstacle(4,:) = [350, 0, 1000, 25];
map.obstacle(5,:) = [650, 225, 675, 975];
map.obstacle(6,:) = [975, 25, 1000, 950];

%map2
% map.obstacle(1,:) =  [0, 50, 100, 1000];
% map.obstacle(2,:) = [100, 975, 700, 1000];
% map.obstacle(3,:) = [300, 0, 400, 775];
% map.obstacle(4,:) = [400, 0, 1000, 25];
% map.obstacle(5,:) = [600, 225, 700, 975];
% map.obstacle(6,:) = [900, 25, 1000, 950];

%map3
% map.obstacle(1,:) =  [0, 50, 25, 1000];
% map.obstacle(2,:) = [25, 975, 675, 1000];
% map.obstacle(3,:) = [325, 0, 350, 775];
% map.obstacle(4,:) = [350, 0, 1000, 25];
% map.obstacle(5,:) = [650, 225, 675, 975];
% map.obstacle(6,:) = [975, 25, 1000, 950];
% map.obstacle(7,:) =  [25, 175, 125, 450];
% map.obstacle(8,:) =  [225, 500, 325, 825];
% map.obstacle(9,:) =  [550, 175, 650, 450];
% map.obstacle(10,:) =  [350, 500, 450, 825];
% map.obstacle(11,:) =  [675, 175, 775, 450];
% map.obstacle(12,:) =  [875, 500, 975, 825];

niter = 10000;
max = 80;

q_start= [0 0];%[20 25];%[175 50];%
step1 = 1;
q_goal = [999 999];%[920 950];%[850 925];%
step2 =1;

node_start(1,:) = q_start;
node_goal(1,:) = q_goal;
figure
axis([0 map.x_max 0 map.y_max])
%rectangle('Position',[0, 50, 25, 975],'FaceColor',[0.5 0.5 0.5])
%rectangle('Position',[25, 975, 650, 25],'FaceColor',[0.5 0.5 0.5])
%rectangle('Position',[325, 0, 25, 775],'FaceColor',[0.5 0.5 0.5])
%rectangle('Position',[350, 0, 650, 25],'FaceColor',[0.5 0.5 0.5])
%rectangle('Position',[650, 225, 25, 750],'FaceColor',[0.5 0.5 0.5])
%rectangle('Position',[975, 25, 25, 925],'FaceColor',[0.5 0.5 0.5])

for i =1:1:size(map.obstacle,1)%
   rectangle('Position',[map.obstacle(i,1),map.obstacle(i,2),-map.obstacle(i,1)+map.obstacle(i,3),-map.obstacle(i,2)+map.obstacle(i,4)],'FaceColor',[0.5 0.5 0.5])%
end%
hold on%
hold on

path_flag = 0;
%% search for nodes along the path
for i = 1:niter
    q = [floor(rand*map.x_max), floor(rand*map.y_max)]; %sampling path here
    isCollided_1 = isCollided(q,map.obstacle); %check obstacle free
    if isCollided_1 == 0 % if the obstacle free
        [q_a, distance_a] = closestNode(q,node_start); % find the closest node in the tree
        isCollided_2 = noCollision_segment_obstacle(q_a,q,map.obstacle); % check collision between new node and previous node
        distance_b = norm(q-q_goal); %check distance between new node and goal
        isCollided_3 = noCollision_segment_obstacle(q_goal,q,map.obstacle); % check collision between new node and goal
        
        if isCollided_2 == 0 && isCollided_3 ==0 % if qqa not collided
            continue;
        end
        
        if distance_a < max && isCollided_2 == 1 % if qqa not collided, add(q,qa) to node_start
            node_start = [node_start;q];
            start_flag = 1;
            start(step1).node = q;
            start(step1).parent = q_a;
            step1 = step1 +1;
        else
            start_flag = 0;
        end
        
        if distance_b < max && isCollided_3 == 1 % if qqgoal not collided, mark the node
            goal_flag = 1;
        else
            goal_flag = 0;
        end
        
        if start_flag == 1 && goal_flag == 1 % if q connected to qa and qgoal, break the loop
            %disp('Path is found!');
            path_flag = 1;
            break;
        end
        
    else
        continue;
    end
end
%% trace the path
[~,idx_start] = size(start);
numStart = idx_start;
for i = 1:idx_start %store the node value in a NX2 matrix
    node1(i,:) = start(i).node;
end
i = 1;
while idx_start ~=1 %while the idx is not 1
    idx_start = find(ismember(node1,start(idx_start).parent,'row'),1); % find the parent of each node
    path1(i) = idx_start; % store the idx in path
    i = i+1;
    break_point = (start(idx_start).parent == q_start); % break the loop is start is found
    if break_point(1) == 1 && break_point(2) == 1;
        break;
    end
end
%% plot the path
for i = 1:length(path1)-1
    line([node1(path1(i),1) node1(path1(i+1),1)],[node1(path1(i),2) node1(path1(i+1),2)], 'Color', 'r', 'LineWidth', 2)
    hold on
end
line([q_start(1) node1((path1(i+1)),1)],[q_start(2) node1(path1(i+1),2)], 'Color', 'r', 'LineWidth', 2)
hold on
line([start(numStart).node(1) node1(path1(1),1)],[start(numStart).node(2) node1(path1(1),2)], 'Color', 'r', 'LineWidth', 2)
hold on
line([start(numStart).node(1) q_goal(1)],[start(numStart).node(2) q_goal(2)], 'Color', 'r', 'LineWidth', 2)
hold on
%% calculate the path length of RRT
for i = 2:length(path1)
    pathlength = norm(node1(path1(i),:)- node1(path1(i-1),:));
    totallength = pathlength+totallength;
end
totallength
end
toc