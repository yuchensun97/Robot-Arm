function [score, evalsRun, errFlag] = runSingleFKTest(calculateFK, q, testNum, tolerances)
% RUNSINGLEFKTEST This function runs a series of evaluations on a single
%   forward kinematics problem. That is, for a single configuration
%   it evaluates the student's solution. Please note that the score that is
%   output is a measure of tests PASSED, NOT YOUR FINAL GRADE.   
%
% AUTHOR:
%   Gedaliah Knizhnik (knizhnik@seas.upenn.edu), 8/28/19
%
% INPUTS:
%   calculateFK - function handle to student's solution
%   q           - the configuration that is the current test
%   testNum     - number of the test being run
%   tolerances  - tolerances relevant to the evaluation
%
% OUTPUTS:
%   score    - the number of evaluations passed
%   evalsRun - the number of evaluations run
%   errFlag  - boolean variable capturing whether an error occured (1) or
%              not (0)

%% Initialize variables

% Clear the memory of the solution code - this catches you if you use
% the solution code in your work.
clear calculateFK_sol
        
errFlag  = 0;
evalsRun = 8;
score    = zeros(evalsRun,1);

fprintf('----------------------------\n');
fprintf('Test #%d: q = [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]\n\n',[testNum,q])

%% Get the student value 
try
    [jointPositions_stu,T0e_stu] = calculateFK(q);
catch ME
    fprintf('Your function returns the following error:\n\n\t');
    disp(ME.message)
    fprintf('\n\nPlease check your code and try again.\n');

    errFlag = 1;
    return
end

%% Get the solution
try
    
    load 'testsFK.mat' Xs Ts
    
    jointPositions_sol = Xs(:,:,testNum);
    T0e_sol            = Ts(:,:,testNum);

catch ME
    fprintf('Cannot access the solutions...\n\n\t');
    disp(ME.message)
    fprintf('\n\nPlease verify that you are running a clean version\n');
    fprintf('of the solution code. If the problem persists, contact\n');
    fprintf('the teaching team via Piazza.\n');

    errFlag = 1;
    return
end
       
        
%% Print the values to the console

fprintf('Your solution was:\n')

jointPositions_stu
T0e_stu

fprintf('The correct solution is:\n')

jointPositions_sol
T0e_sol
 

%% Run the evaluations
        
try
    %% Calculate the error in joint positions
    fprintf('Testing joint positions:\n\n');

    errJointPositions = jointPositions_sol - jointPositions_stu;
        
    % Format output
    for kk = 1:size(jointPositions_sol,1)
        if norm(errJointPositions(kk,:)) < tolerances.errTolPos
            fprintf('\t Joint %d located correctly.\n',kk)
            score(kk) = 1;
        else
            fprintf('\t Joint %d in error... \n \t\t Error is [%.2f,%.2f,%.2f]\n',...
                    [kk,errJointPositions(kk,1),errJointPositions(kk,2),errJointPositions(kk,3)]);
        end
    end

    fprintf('\n')
    fprintf('Testing the homogeneous transform:\n\n');
                
    %% Calculate the error in final position

    errFin = T0e_sol - T0e_stu;
    errFinPos = errFin(1:3,4);

    if norm(errFinPos) < tolerances.errTolPos
        fprintf('\t Position of final joint in h. transform is correct.\n')
        score(7) = 1;
    else
        fprintf(['\t Position of final joint in h. transform is in error...\n',...
                 '\t\t Error is [%.2f,%.2f,%.2f]\n'],[errFinPos(1),errFinPos(2),errFinPos(3)])
    end

    fprintf('\n');

    %% Calculate the error in final orientation

    % Extract just the rotation matrices
    R0e_sol = T0e_sol(1:3,1:3);
    R0e_stu = T0e_stu(1:3,1:3);
    
    % Calculate the error in the rotation matrix
    errRot0e = 1/2*veemap(R0e_sol'*R0e_stu - R0e_stu'*R0e_sol);

    if norm(errRot0e) < tolerances.errTolOri
        fprintf('\t Orientation of final joint is correct.\n');
        score(8) = 1;
    else
        fprintf(['\t Orientation of final joint is in error...\n',...
                 '\t\t Error is [%.2f,%.2f,%.2f]\n'],[errRot0e(1),errRot0e(2),errRot0e(3)])
    end

    fprintf('\n')
        
catch ME
    fprintf('Your function returns the following error:\n\n\t');
    disp(ME.message)
    fprintf('\n\nPlease check your code and try again.\n');
end
                
end

% Map the rotation matrix error from skew symmetric to a vector
function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end