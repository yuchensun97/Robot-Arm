% This function returns an optimized tree
function back_optimize = PathOptimization(path)
% Input:
% Output:
%% Algorithms starts here
while(1)
    flag=0;
    back_optimize=path(count+1);
    while path(back_optimize.parent).parent>1
        if (noCollision_segment_obstacle(path(path(back_optimize.parent).parent).workspace,back_optimize.workspace,obstacles))
            flag=1;
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