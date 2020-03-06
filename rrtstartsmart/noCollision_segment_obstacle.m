function noCollision = noCollision_segment_obstacle(n2, n1, obstacle)
%     noCollision=1, no collide
    flag=0;
%     obstacle = [obstacle(1) obstacle(2) obstacle(1)+obstacle(3) obstacle(2)+obstacle(4)];
    for j=1:1:size(obstacle,1)
% Check if segment formed by n1 to n2 intersects any of the four edges
% of the obstacle
        obs_point=[[obstacle(j,1),obstacle(j,2)];[obstacle(j,3),obstacle(j,2)];[obstacle(j,3),obstacle(j,4)];[obstacle(j,1),obstacle(j,4)];...
            [obstacle(j,1),obstacle(j,2)]];
        for i=1:1:4
            intersect = detect_same_side(n1,obs_point(i,:),obs_point(i+1,:)) ~= detect_same_side(n2,obs_point(i,:),obs_point(i+1,:)) &&...
                detect_same_side(n1,n2,obs_point(i,:)) ~= detect_same_side(n1,n2,obs_point(i+1,:)); 
            if intersect==1
                flag=1;
                break
            end
        end
    end
    if flag==0
        noCollision = 1;
    else
        noCollision = 0;
    end
end