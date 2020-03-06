function [jointPositions,T0e] = calculateFK_13(q)
% CALCULATEFK_PENNKEY - Please rename this function using your pennkey in
%   both the function header and the file name. If you are working in
%   pairs, choose ONE pennkey to use - otherwise the grading scripts will
%   get confused.
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 6 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0e            - a 4 x 4 homogeneous transformation matrix, 
%                    representing the end effector frame expressed in the 
%                    base (0) frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here
%% Robot's parameter
d1 = 3*25.4; % offset from base origin to joint 1, in mm
a2 = 5.75*25.4; % distance between joint 1 and joint 2, in mm
a3 = 7.375*25.4; % distance between joint 2 and joint 3 (wrist center), in mm
d5 = 3*25.4; % offset from joint 3 (wrist center) to joint 5 (gripper center), in mm
lg = 1.125*25.4; % the length of end effector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% DH Parameter
a = [0 a2 a3 0 0]; % distance between each joint
alpha = [-pi/2 0 0 -pi/2 0]; % rotation of z axis
d = [d1 0 0 0 d5]; %offset of each 
q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5);
theta = [q1 q2-pi/2 q3+pi/2 q4-pi/2 q5];
p4 = [0;0;34;1]; % position of joint 4 in frame 4
A6 =[1 0 0 0;
     0 1 0 0;
     0 0 1 lg;
     0 0 0 1;]; % transformation matrix from frame 5 to frame e
%% Compute each transformation matrices
A = cell(1,5); % create a cell to store all the transformation matrices
T = eye(4); % create a matrics to store final transformation matrices
pos = [0;0;0;1]; % vector to extract each joint's position
origin = [0 0 0]; % position of the origin joint.
jointPositions(1:3,1) = origin; % store the position of the origin
for i = 1:5 
    A{1, i} = [cos(theta(i)) -sin(theta(i))*cos(alpha(i)) sin(theta(i))*sin(alpha(i)) a(i)*cos(theta(i));
               sin(theta(i)) cos(theta(i))*cos(alpha(i)) -cos(theta(i))*sin(alpha(i)) a(i)*sin(theta(i));
               0         sin(alpha(i))            cos(alpha(i))                    d(i);
               0         0                        0                                 1]; %compute transformation matrix related to last frame
    T = T*A{1,i}; %compute each frame's final transformation matrices
    position = T*pos; % extract the position of each joint
    if i == 4
        p4 = T*p4; % compute the position of joint 4
        jointPositions(1:3, i+1) = p4(1:3,1); %store each joint's position in jointPosition matrices
    else
        jointPositions(1:3, i+1) = position(1:3,1); %store each joint's position in jointPosition matrices
    end
end
T0e = T*A6;   % end effector frame expressed in the base (0) frame
jointPositions = jointPositions';
end