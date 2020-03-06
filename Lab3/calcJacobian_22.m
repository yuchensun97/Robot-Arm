function J = calcJacobian_22(q, joint, robot)
% CALCJACOBIAN_GROUPNO Calculate the Jacobian of a particular joint of the 
%   robot in a given configuration. CHANGE GROUPNO TO YOUR GROUP NUMBER.
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   joint - scalar in [1,7] representing which joint we care about
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   J - 6 x (joint-1) matrix representing the Jacobian
%

%%
J = [];

if nargin < 3
    return
elseif joint <= 1 || joint > 7
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

J = zeros(6,joint-1);
%% get the transformation matrix of the robot
[jointPos,T0i] = calculateFK_sol(q);
T0 = T0i(:,:,1);
T1 = T0i(:,:,2);
T2 = T0i(:,:,3);
T3 = T0i(:,:,4);
T4 = T0i(:,:,5);
T5 = T0i(:,:,6);
T6 = T0i(:,:,7);
T = cell(1,6); % create a cell to store all the transformation matrix
T={T1,T2,T3,T4,T5,T6};
z0 = [0;0;1];
%% get the jointPosition of the robot
o0 = (jointPos(1,:))';
o1 = (jointPos(2,:))';
o2 = (jointPos(3,:))';
o3 = (jointPos(4,:))';
o4 = (jointPos(5,:))';
o5 = (jointPos(6,:))';
o6 = (jointPos(7,:))';
o = [o1 o2 o3 o4 o5 o6];
%% get the Jacobian using geometry method
if joint == 1
    J = [];
else
    Jv(:,1) = cross(z0,o(:,joint-1)-o0);% calculate the first colum of velocity Jacobian
    Ja(:,1) = z0; % calculate the first colum of angular velocity
    for i=1:joint-2
        z(:,i) = T{1,i}(1:3,1:3)*z0; % get the z axis of each frame
        Jv(:,i+1)= cross(z(:,i),o(:,joint-1)-o(:,i)); % get the velocity Jacobian
        Ja(:,i+1)= z(:,i); % get the angular Jacobian
    end
    for i=1:joint-1 %get the Jacobian
        J(1:3,i)=Jv(:,i);
        J(4:6,i)=Ja(:,i);
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end