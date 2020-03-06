function [qnew min_distance] = closestNode(q,node);
% This function find the closest node in the set of node
%   Inputt:
%   q -- a 1X2 vector represent the current node
%   node -- set of nodes
%   Output:
%   qnew -- closest node in the set node
%   distance -- the distance between two node
%% Algorithm starts here
% clc
% clear
% node = [20 20];
% q = [257 785];
[numNode,~] = size(node);
for i = 1:numNode
    Distance(i)=norm(q-node(i,:));
end
[min_distance,index]= min(Distance);
qnew = node(index,:);
end