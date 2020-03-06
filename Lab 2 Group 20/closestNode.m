function [q_a,distancemin,min_pos]=closestNode(T_start,new_q)
%This function find the closest node in T_start
%Input:
%      T_start: A NX6 matrix that store the previous C-space
%      new_q: A 1X6 matrix that store the C- space of
%      the new random point.
%Output:
%      q_a: A C-SPACE from T_start, which is closest to new_q.
%      distancemin: The minimum distance between pos_a and T_start. 
%      min_pos: The position of the minimum distance in T_spos
%% Algorithm Starts Here
[r,c]=size(T_start);
for i = 1:r
    %[J1,T_pre]=calculateFK_sol(T_start(i,:));
    %[J2,T_post]=calculateFK_sol(new_q);
    %distance(i) = sqrt((T_pre(1,4)-T_post(1,4))^2+(T_pre(2,4)-T_post(2,4))^2+(T_pre(3,4)-T_post(3,4))^2); %calculate distances between points in T_spos and new_q_pos
   % distance(i) = norm(T_start(i,:) - new_q);
    distance(i) = sqrt((T_start(i,1)- new_q(1))^2+(T_start(i,2)- new_q(2))^2+(T_start(i,3)- new_q(3))^2+(T_start(i,4)- new_q(4))^2+(T_start(i,5)- new_q(5))^2+(T_start(i,6)- new_q(6))^2);
end
min_pos = find(distance==min(distance)); %find the position of minimum distance
q_a = T_start(min_pos,:); %get the closest node
distancemin = min(distance);%get the minimum distance
%% Algorithm Ends
end