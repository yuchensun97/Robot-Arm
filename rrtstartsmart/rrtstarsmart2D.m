tic
% for alltime=1:1:20
close all; 
clear all;


x_max = 1000;
y_max = 1000;

obstacle1 =  [0, 50, 25, 1000];
obstacle2 = [25, 975, 675, 1000];
obstacle3 = [325, 0, 350, 775];
obstacle4 = [350, 0, 1000, 25];
obstacle5 = [650, 225, 675, 975];
obstacle6 = [975, 25, 1000, 950];

numNodes = 1500;  
nearD = 50;
max = 30; 

b=20;
beacon_step = 0;
global obstacles
obstacles = [obstacle1; obstacle2; obstacle3; obstacle4; obstacle5; obstacle6];

start.workspace = [50, 25];
start.cost = 0;
start.parent = 0;
goal.workspace = [920, 975];
goal.cost = 0;
goal.parent=0;
qrand.workspace=[0 0];
qrand.cost = 0;
qrand.parent=0;

plot(start.workspace(1), start.workspace(2),'c*',goal.workspace(1), goal.workspace(2),'c*')
hold on

path(1) = start;
qrand_all = [rand(numNodes,1)*x_max,rand(numNodes,1)*y_max];
distance=[];

for i =1:1:size(obstacles,1)
    rectangle('Position',[obstacles(i,1),obstacles(i,2),-obstacles(i,1)+obstacles(i,3),-obstacles(i,2)+obstacles(i,4)],'FaceColor',[0.5 0.5 0.5])
end


for j = 1:1:numNodes
    qrand.workspace=qrand_all(j,:);
    distance= vecnorm(cat(1,path.workspace)-ones(size(cat(1,path.workspace),1),2).*qrand.workspace,2,2);
   
    [minD,Ind] = min(distance);
    q_closest = path(Ind);

    if minD > max
        qrand.workspace = [q_closest.workspace(1) + (qrand.workspace(1)-q_closest.workspace(1))*max/minD,q_closest.workspace(2) + (qrand.workspace(2)-q_closest.workspace(2))*max/minD];
    end
    if noCollision_segment_obstacle(qrand.workspace,q_closest.workspace,obstacles) 
        qrand.parent = Ind;          
        qrand.cost = norm(qrand.workspace- q_closest.workspace) + q_closest.cost;
        for i = 1:length(path)
            if (norm(qrand.workspace-path(i).workspace) < nearD)
                if (path(i).cost + norm(qrand.workspace-path(i).workspace)< qrand.cost)
                    if (noCollision_segment_obstacle(qrand.workspace,path(i).workspace,obstacles))
                        qrand.cost = path(i).cost + norm(qrand.workspace-path(i).workspace);
                        qrand.parent = i;
                    end
                end
            end
        end 
        path = [path qrand];
    end
end

Tmpdist=vecnorm(cat(1,path.workspace)-ones(size(cat(1,path.workspace),1),2).*goal.workspace,2,2);

%find the closest point to goal
while goal.parent==0
    [val, idx] = min(Tmpdist);
    if noCollision_segment_obstacle(path(idx).workspace,goal.workspace,obstacles)
        final = path(idx);
        goal.parent = idx;
        goal.cost = norm(goal.workspace- path(idx).workspace) + path(idx).cost
    else
        Tmpdist(idx)=1000;
    end
end
    
path = [path goal];

back = goal;
final_path=goal.workspace;
while back.parent ~= 0
    iter = back.parent;
    line([back.workspace(1), path(iter).workspace(1)], [back.workspace(2), path(iter).workspace(2)], 'Color', 'r', 'LineWidth', 2);
    final_path=[final_path;path(iter).workspace];
    back = path(iter);
end
final_path=flipud(final_path);

newsize = size(final_path)/2;

for i = 1:newsize
    firstPath(i) = final_path(2*i - 1);
end

optSize = size(path);
hold off
toc