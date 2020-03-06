%% Setup
clc
clear
close all
addpath('utils')
addpath('maps')

profile on

%% Simulation Parameters
%
% Define any additional parameters you may need here. Add the parameters to
% the robot and map structures to pass them to astar or rrt.
%
load 'robot.mat' robot

start = [0,0,0,0,0,0];
goal = [1.4,0,0,0,0,0];

map = loadmap('map1.txt');


%% Run the simulation

% Solve the path problem using A*
%[path, num] = astar();

% OR Solve the path problem using RRT
[path] = rrt(map,start,goal);

profile off

%% Plot the output

plotLynxPath(map,path,10);

profile viewer
