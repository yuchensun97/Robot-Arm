function collided = isCollided(qrand, obstables)

collided = 0;
%collision=0;
a = length(obstables);

for i = 1:a
    if ( (qrand(1) > obstables(i,1) && qrand(1) < obstables(i,3)) && (qrand(2) > obstables(i,2) && qrand(2) < obstables(i,4)))
        collided = 1;
        break
    end
end
    
end