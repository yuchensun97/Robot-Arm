close all; clear all;
tic

profile on
x_max = 1000;
y_max = 1000;

obstacle1 =  [0, 50, 25, 1000];
obstacle2 = [25, 975, 675, 1000];
obstacle3 = [325, 0, 350, 775];
obstacle4 = [350, 0, 1000, 25];
obstacle5 = [650, 225, 675, 975];
obstacle6 = [975, 25, 1000, 950];
numNodes = 2000;  
nearD = 50;
max = 30; 

obstacles = [obstacle1; obstacle2; obstacle3; obstacle4; obstacle5; obstacle6];

start.config = [20, 25];
start.cost = 0;
start.parent = 0;
goal.config = [920, 950];
goal.cost = 0;
goal.parent=0;
qrand.config=[0 0];
qrand.cost = 0;
qrand.parent=0;

plot(start.config(1), start.config(2),'c*',goal.config(1), goal.config(2),'c*')

path(1) = start;
path_config=start.config;
axis([0 x_max 0 y_max])
qrand_all = [rand(numNodes,1)*x_max,rand(numNodes,1)*y_max];
distance=[];

for i =1:1:size(obstacles,1)
    rectangle('Position',[obstacles(i,1),obstacles(i,2),-obstacles(i,1)+obstacles(i,3),-obstacles(i,2)+obstacles(i,4)],'FaceColor',[0.5 0.5 0.5])
end
hold on

for j = 1:1:numNodes
    qrand.config=qrand_all(j,:);
    distance= vecnorm(path_config-ones(size(path_config,1),2).*qrand.config,2,2);
   
    [minD,Ind] = min(distance);
    q_closest = path(Ind);

    if minD > max
        qrand.config = [q_closest.config(1) + (qrand.config(1)-q_closest.config(1))*max/minD,q_closest.config(2) + (qrand.config(2)-q_closest.config(2))*max/minD];
    end
    if noCollision_segment_obstacle(qrand.config,q_closest.config,obstacles) 
        qrand.parent = Ind;         
        qrand.cost = norm(qrand.config- q_closest.config) + q_closest.cost;
        for i = 1:length(path)
            if (norm(qrand.config-path(i).config) < nearD)
                if (path(i).cost + norm(qrand.config-path(i).config)< qrand.cost)
                    if (noCollision_segment_obstacle(qrand.config,path(i).config,obstacles))
                        qrand.cost = path(i).cost + norm(qrand.config-path(i).config);
                        qrand.parent = i;
                    end
                end
            end
        end 
        path = [path qrand];
        path_config=[path_config;qrand.config];
     end

end

Tmpdist=[];
Tmpdist=vecnorm(path_config-ones(size(path_config,1),2).*goal.config,2,2);

while goal.parent==0
    [val, idx] = min(Tmpdist);
    if noCollision_segment_obstacle(path(idx).config,goal.config,obstacles)
        final = path(idx);
        goal.parent = idx;
    else
        Tmpdist(idx)=1000;
    end
end
    
path = [path goal];

back = goal;
final_path=goal.config;
while back.parent ~= 0
    iter = back.parent;
    line([back.config(1), path(iter).config(1)], [back.config(2), path(iter).config(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    final_path=[final_path;path(iter).config];
    back = path(iter);
end
final_path=flipud(final_path);
profile viewer
toc