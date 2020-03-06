tic
% for alltime=1:1:20
% profile on
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


% obstacle1 =  [0, 50, 25, 1000];
% obstacle2 = [25, 975, 675, 1000];
% obstacle3 = [325, 0, 350, 775];
% obstacle4 = [350, 0, 1000, 25];
% obstacle5 = [650, 225, 675, 975];
% obstacle6 = [975, 25, 1000, 950];
% obstacle7 =  [25, 175, 125, 450];
% obstacle8 =  [225, 500, 325, 825];
% obstacle9 =  [550, 175, 650, 450];
% obstacle10 =  [350, 500, 450, 825];
% obstacle11 =  [675, 175, 775, 450];
% obstacle12 =  [875, 500, 975, 825];


numNodes = 1500;  
nearD = 50;
max = 30; 
path_found_radius=500;
sample_radius=100;

b=5;
n = numNodes+1;

% obstacles = [obstacle1; obstacle2; obstacle3; obstacle4; obstacle5; obstacle6;
%     obstacle7; obstacle8; obstacle9; obstacle10; obstacle11; obstacle12];
obstacles = [obstacle1; obstacle2;  obstacle3;obstacle4; obstacle5; obstacle6];


start.workspace = [175, 50];
start.cost = 0;
start.parent = 1;
start.idx=1;
goal.workspace = [825, 950];
goal.cost = 0;
goal_distance = 100000;
goal.parent=0;
qrand.workspace=[0 0];
qrand.cost = 0;
qrand.parent=0;

plot(start.workspace(1), start.workspace(2),'c*',goal.workspace(1), goal.workspace(2),'c*')
hold on

path(1) = start;
qrand_all = [rand(numNodes,1)*x_max,rand(numNodes,1)*y_max];
beacons_rand_all=[rand(numNodes,4)];
distance=[];
path_found=0;
path_found_sample=0;
count=1;
total_cost=10000;
repeat_time=30;
final_path_length=0;

for i =1:1:size(obstacles,1)
    rectangle('Position',[obstacles(i,1),obstacles(i,2),-obstacles(i,1)+obstacles(i,3),-obstacles(i,2)+obstacles(i,4)],'FaceColor',[0.5 0.5 0.5])
end

for r=1:1:repeat_time
    r
    color_path=rand([1,3]);
    for j = 1:1:numNodes

    %     j
        if path_found_sample && mod(j,b)==0
            m = randi([1 length(beacons)],1);
            qrand.workspace=[beacons(m,1)+beacons_rand_all(j,1)*sample_radius*cos(beacons_rand_all(j,2)*2*pi),...
                beacons(m,2)+beacons_rand_all(j,3)*sample_radius*sin(beacons_rand_all(j,4)*2*pi)];
%             plot(qrand.workspace(1), qrand.workspace(2),'b+')

        else
            qrand.workspace=qrand_all(j,:);
        end
    %     path_found=0;
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

%             line([q_closest.workspace(1), qrand.workspace(1)], [q_closest.workspace(2), qrand.workspace(2)], 'Color',[0.4660 0.6740 0.1880]);
            line([q_closest.workspace(1), qrand.workspace(1)], [q_closest.workspace(2), qrand.workspace(2)], 'Color',color_path);
            % every step
            drawnow
            hold on            
            qrand.cost = norm(qrand.workspace- q_closest.workspace) + q_closest.cost;

            % find the shortest path within nearD
            path(count) =qrand;
            if path_found_sample==0
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
            end
        end
    end
    Tmpdist=vecnorm(cat(1,path.workspace)-ones(size(cat(1,path.workspace),1),2).*goal.workspace,2,2);

    % find the closest point to goal
    while goal.parent==0
        [~, index] = min(Tmpdist);
        if noCollision_segment_obstacle(path(index).workspace,goal.workspace,obstacles)
    %         final = path(idx);
            goal.parent = index;
            goal.cost = norm(goal.workspace- path(index).workspace) + path(index).cost;

        else
            Tmpdist(index)=1000;
        end
    end

    if norm(path(goal.parent).workspace-goal.workspace)<path_found_radius
        path_found=1;
        goal.idx=count+1;
    end
    %         goal_tmp=goal;
    if path_found
        path(count+1)=goal;
        path_found_sample=1;
    %             for k=1:1:length(path)
        while(1)
            flag=0;
            back_optimize=path(count+1);

            while back_optimize.parent>1
                flag=0;
                if (noCollision_segment_obstacle(path(path(back_optimize.parent).parent).workspace,back_optimize.workspace,obstacles))
                    flag=1;
    %                     line([path(path(back_optimize.parent).parent).workspace(1), back_optimize.workspace(1)],[path(path(back_optimize.parent).parent).workspace(2), back_optimize.workspace(2)], 'Color', color, 'LineWidth', 2);
    %                     drawnow
%                     path(back_optimize.idx).cost = path(path(back_optimize.parent).parent).cost + norm( path(back_optimize.idx).workspace-path(path(back_optimize.parent).parent).workspace);
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
        calculate_cost=0;
        back_calculate_cost=path(count+1);
        while back_calculate_cost.workspace ~= start.workspace
            calculate_cost = calculate_cost+ norm( back_calculate_cost.workspace-path(back_calculate_cost.parent).workspace);
            back_calculate_cost=path(back_calculate_cost.parent);
        end
        if calculate_cost<total_cost
            clear final_path
    %         clear new_path
            total_cost=calculate_cost;
            back = path(count+1);
            final_path=path(count+1).workspace;
            goal.cost=calculate_cost
    %         new_path_tmp(1)=path(count+1);
    %             goal.parent=path(count+1).parent
            color=rand([1,3]);
            while back.workspace ~= start.workspace
                iter = back.parent;
                final_path=[final_path;path(iter).workspace];
    %             new_path_tmp(n)=path(iter);                
                line([back.workspace(1), path(iter).workspace(1)], [back.workspace(2), path(iter).workspace(2)], 'Color',color, 'LineWidth', 0.5);
                drawnow 
                hold on
                back = path(iter);
            end
            beacons=flipud(final_path);
%             q=size(beacons,1);
    %         for p=1:1:size(beacons,1)
    %             new_path(p)=new_path_tmp(q);
    %             q=q-1;
    %         end
        end
    %             path=new_path;
    %         goal.cost=0;
    %         goal.parent=0;
    end
	clear path
	path(1) = start;
    count=1;
	path_found=0;
    goal.parent=0;
end

hold off
% profile off
% profile viewer
% end

toc