% tic
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
numNodes = 1000;  
nearD = 50;
max = 30; 
path_found_radius=100;

% obstacles = [obstacle1; obstacle2; obstacle3; obstacle4; obstacle5; obstacle6];
obstacles = [obstacle1; obstacle2;  obstacle3;obstacle4;  obstacle6];
start.workspace = [20, 25];
start.cost = 0;
start.parent = 0;
start.idx=1;
goal.workspace = [920, 950];
goal.cost = 0;
goal_distance = 10000;
goal.parent=0;
qrand.workspace=[0 0];
qrand.cost = 0;
qrand.parent=0;

plot(start.workspace(1), start.workspace(2),'c*',goal.workspace(1), goal.workspace(2),'c*')
hold on

path(1) = start;
qrand_all = [rand(numNodes,1)*x_max,rand(numNodes,1)*y_max];
distance=[];
path_found=0;
count=1;

for i =1:1:size(obstacles,1)
    rectangle('Position',[obstacles(i,1),obstacles(i,2),-obstacles(i,1)+obstacles(i,3),-obstacles(i,2)+obstacles(i,4)],'FaceColor',[0.5 0.5 0.5])
end


for j = 1:1:numNodes
%     j
    goal_changed=0;
    if goal.parent == 0
        qrand.workspace=qrand_all(j,:);
        distance= vecnorm(cat(1,path.workspace)-ones(size(cat(1,path.workspace),1),2).*qrand.workspace,2,2);

        [minD,Ind] = min(distance);
        q_closest = path(Ind);

        if minD > max
            qrand.workspace = [q_closest.workspace(1) + (qrand.workspace(1)-q_closest.workspace(1))*max/minD,q_closest.workspace(2) + (qrand.workspace(2)-q_closest.workspace(2))*max/minD];
    %     qrand.workspace = shorterD(qrand.workspace, q_closest.workspace, minD, max); 
        end
        if noCollision_segment_obstacle(qrand.workspace,q_closest.workspace,obstacles) 
            count=count+1;
            qrand.idx=count;
            qrand.parent = Ind;
    %         line([q_closest.workspace(1), qrand.workspace(1)], [q_closest.workspace(2), qrand.workspace(2)], 'Color',[0.4660 0.6740 0.1880]);
    %         drawnow
    %         hold on            
            qrand.cost = norm(qrand.workspace- q_closest.workspace) + q_closest.cost;

            % find the shortest path within nearD
            for i = 1:length(path)
                if (norm(qrand.workspace-path(i).workspace) < nearD)
                    if (path(i).cost + norm(qrand.workspace-path(i).workspace)< qrand.cost)
                        if (noCollision_segment_obstacle(qrand.workspace,path(i).workspace,obstacles))
                            qrand.cost = path(i).cost + norm(qrand.workspace-path(i).workspace);
                            qrand.parent = i;
    %                         line([path(i).workspace(1), qrand.workspace(1)], [path(i).workspace(2), qrand.workspace(2)], 'Color', [0.3010 0.7450 0.9330]);
    %                         hold on
                        end
                    end
                end
            end 
            path(count) =qrand;

            if norm(qrand.workspace-goal.workspace)<path_found_radius
                path_found=1;
                if norm(goal.workspace- qrand.workspace) <goal_distance
                    goal.parent=count;
                    goal.idx=count+1;
                    goal_distance= norm(goal.workspace- qrand.workspace);
                    goal_changed=1;
                end
            end
    %         goal_tmp=goal;
            if path_found
                path(count+1)=goal;
                n = j;
                while(1)
                    flag=0;
                    back_optimize=path(count+1);
                    color=rand([1,3]);
                    while path(back_optimize.parent).parent>1
                        if (noCollision_segment_obstacle(path(path(back_optimize.parent).parent).workspace,back_optimize.workspace,obstacles))
                            flag=1;
                            line([path(path(back_optimize.parent).parent).workspace(1), back_optimize.workspace(1)],[path(path(back_optimize.parent).parent).workspace(2), back_optimize.workspace(2)], 'Color', color, 'LineWidth', 2);
                            drawnow
                            path(back_optimize.idx).parent=path(back_optimize.parent).parent;
                            back_optimize=path(path(back_optimize.idx).parent);
                        else
                            back_optimize=path(path(back_optimize.parent).parent);                        
                        end
                    end
                    if flag==0
                        break
                    end  
                end
            end
        end
    end
end

Tmpdist=vecnorm(cat(1,path.workspace)-ones(size(cat(1,path.workspace),1),2).*goal.workspace,2,2);

% find the closest point to goal
while goal.parent==0
    [~, idx] = min(Tmpdist);
    if noCollision_segment_obstacle(path(idx).workspace,goal.workspace,obstacles)
%         final = path(idx);
        goal.parent = idx;
        goal.cost = norm(goal.workspace- path(idx).workspace) + path(idx).cost;
    else
        Tmpdist(idx)=1000;
    end
end
    
% path = [path goal];

back = goal;
final_path=goal.workspace;
while back.parent ~= 0
    iter = back.parent;
    line([back.workspace(1), path(iter).workspace(1)], [back.workspace(2), path(iter).workspace(2)], 'Color', 'r', 'LineWidth', 2);
%     hold on
    final_path=[final_path;path(iter).workspace];
    back = path(iter);
end
final_path=flipud(final_path);
% profile viewer
% end
hold off