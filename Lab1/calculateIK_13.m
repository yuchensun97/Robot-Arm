function [q isPos] = calculateIK_13(T0e)
% CALCULATEIK_PENNKEY - Please rename this function using your pennkey in
%   both the function header and the file name. If you are working in
%   pairs, choose ONE pennkey to use - otherwise the grading scripts will
%   get confused.
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)
%
% OUTPUT:
%   q          - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) 
%                which are required for the Lynx robot to reach the given 
%                transformation matrix T. Each row represents a single
%                solution to the IK problem. If the transform is
%                infeasible, q should be all zeros.
%   isPos      - a boolean set to true if the provided
%                transformation T is achievable by the Lynx robot, ignoring
%                joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here
%% initialization here
L0 = 3*25.4;
L1 = 5.75*25.4;
L2 = 7.375*25.4;
L34 = 4.125*25.4;
T = T0e;
r11 = T(1,1);
r12 = T(1,2);
r13 = T(1,3);
r21 = T(2,1);
r22 = T(2,2);
r23 = T(2,3);
r31 = T(3,1);
r32 = T(3,2);
r33 = T(3,3);
R = T(1:3, 1:3);
ox = T(1,4);
oy = T(2,4);
oz = T(3,4);
XYZ = [ox;oy;oz];
%% determine if T0e is in the workspace regardless of joint limits
R0 = L0+L1+L2+L34;
robotR = sqrt(ox^2+oy^2+oz^2);
q = [];
NaN = 0;   
if robotR>R0
    isPos = 0;
else
    isPos = 1;
    n = 1;
    s = 0;
    %% determin if the orientation is feasible
    Orientation = [r13 r23 r33;ox oy oz;0 0 1];
    DETERMINANT = det(Orientation);
    xyz = XYZ - L34*R*[0;0;1];
    x = xyz(1,1);
    y = xyz(2,1);
    z = xyz(3,1);
    if DETERMINANT == 0
    else % if the orientation is infeasible
        theta =  atan2(y,x) - atan2(oy,ox);
        T = T*[1 0 0 0;0 cos(theta) -sin(theta) 0; 0 sin(theta) cos(theta) 0;0 0 0 1]; %calcute the new T0e
        isPos = 0;
        if theta > 0.00000000001 | theta < -0.00000000001
            n = 0;
        end
    end
    if ox == 0 & oy == 0 % if there are infinite solutions
        NaN = 1; % set NaN to 1.
    end
    %% re-define the transformation matrice
    r11 = T(1,1);
    r12 = T(1,2);
    r13 = T(1,3);
    r21 = T(2,1);
    r22 = T(2,2);
    r23 = T(2,3);
    r31 = T(3,1);
    r32 = T(3,2);
    r33 = T(3,3);
    R = T(1:3, 1:3);
    ox = T(1,4);
    oy = T(2,4);
    oz = T(3,4);
    XYZ = [ox;oy;oz];
    xyz = XYZ - L34*R*[0;0;1];
    x = xyz(1,1);
	y = xyz(2,1);
    z = xyz(3,1);
    %% calculation for joint variables
    if NaN == 1;
        theta11 = 0; %for later calculation
        theta12 = pi;%for later calculation
    end
    theta11 = atan2(y,x); % calculate q1
    theta12 = atan2(y,x)+pi; % calculate another solution to q1
    if theta12 > pi %ensure q1 lies in [-pi,pi]
        theta12 = theta12-2*pi;
    end
    k = (-(z - L0)^2 - x^2 - y^2 + L1^2 + L2^2)/(2*L1*L2);
    theta31 = atan2(k,((1-k^2)^0.5)); %calculate q3
    theta32 = atan2(k,-((1-k^2)^0.5)); %calculate another solutions to q3
    theta1 = [theta11 theta12];
    theta3 = [theta31 theta32];
    % below loop calculate q2 q4 q5
    for i = theta1
        for j = theta3
            theta2 = atan2((x^2+y^2)^0.5,z-L0)-atan2(L2*cos(j), L1 - L2*sin(j)); % calculate q2
            theta4 = atan2(-r33*cos(theta2+j)-r13*sin(theta2+j)*cos(i)-r23*sin(theta2+j)*sin(i),...
            r13*cos(theta2+j)*cos(i)-r33*sin(theta2+j)+r23*cos(theta2+j)*sin(i)); % calculate q4
            theta5 = atan2(-r21*cos(i)+r11*sin(i),-r22*cos(i)+r12*sin(i)); %calculate q5
            angles = [i theta2 j theta4 theta5];
            % use joint limit to narrow down solutions
            if i < 1.4 & i > -1.4 & theta2>-1.2 & theta2<1.4 & j>-1.8 & j< 1.7 & theta4>-1.9 & theta4<1.7 & theta5>-2 & theta5<1.5
                a = [angles];
                q = [q;a];
                if n == 1
                   isPos = 1;
                   s = 1;
                end
            else
                c = [];
                q = [q;c];
                if isPos == 0 & s == 0
                    isPos = 0;
                end
            end
        end
   end
end
if NaN == 1
    q(1) = 'NaN'; q(5) = 'NaN'; % mark the unconstraint joints
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%