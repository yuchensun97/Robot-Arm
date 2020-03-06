function [score, evalsRun, errFlag] = runSingleIKTest(calculateIK,T,testNum,tolerances)
% RUNSINGLEIKTEST This function runs a series of evaluations on a single
%   inverse kinematics problem. That is, for a single homogeneous transform
%   it evaluates the student's solution. Please note that the score that is
%   output is a measure of tests PASSED, NOT YOUR FINAL GRADE. 
%
%   Your calculateIK function should return all the possible solutions. If
%   you return multiple solutions that are the same, they will not be
%   considered separately.
%
% AUTHOR:
%   Gedaliah Knizhnik (knizhnik@seas.upenn.edu), 8/28/19
%
% INPUTS:
%   calculateIK - function handle to student's solution
%   T           - the homogeneous transformation that is the current test
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
clear calculateIK_sol
        
errFlag  = 0;
evalsRun = 3;
score    = zeros(evalsRun,1);

fprintf('----------------------------\n');
fprintf('Test #%d: the input is: \n', testNum);
        
T
 
%% Get the student value 
try
    [q_stu,isPos_stu] = calculateIK(T);
    q_stu = unique(q_stu,'rows'); % Only use unique solutions
catch ME
    fprintf('Your function returns the following error:\n\n\t');
    disp(ME.message)
    fprintf('\nPlease check your code and try again.\n');

    errFlag = 1;
    return
end
         
%% Get the solution
try
    
    load 'testsIK.mat' qs isPoss
    q_sol = qs{testNum};
    isPos_sol = isPoss(testNum);

    if isPos_sol
        dispString = 'feasible';
    else
        dispString = 'infeasible';
    end

catch ME
    fprintf('Cannot access the solutions...\n\n\t');
    disp(ME.message)
    fprintf('\nPlease verify that you are running a clean version\n');
    fprintf('of the solution code. If the problem persists, contact\n');
    fprintf('the teaching team via Piazza.\n');

    errFlag = 1;
    return
end
       
        
%% Print the values to the console

fprintf('Your solution was:\n')

q_stu
isPos_stu 

fprintf('The correct solution is:\n')

q_sol
isPos_sol
    


%% Run the evaluations

try
    %% Is the feasibility correctly identified?
    fprintf('Testing feasibility:\n\n')

    errIsPos = ~(isPos_stu == isPos_sol);            
    score(1) = ~errIsPos;

    if ~errIsPos
        fprintf('\t You are correct. The transform is %s\n',...
            dispString);
    else
        fprintf('\t You are incorrect. The transform is %s\n',...
            dispString);
    end

    %% Is the number of solutions the same?
    fprintf('\nTesting number of solutions:\n\n')

    numSolSame = (size(q_stu,1) == size(q_sol,1));
    score(2) = numSolSame;

    if numSolSame
        fprintf('\t You are correct. There are %d solution(s)\n',...
            size(q_sol,1));
    else
        fprintf('\t You are incorrect. There should be %d solution(s)\n',...
            size(q_sol,1));
    end

    %% Are the solutions the same?
    fprintf('\nTesting the solutions:\n\n')

    numRight = 0;

    for ii = 1:size(q_stu,1)
        isSol = vecnorm(wrapToPi(q_sol - q_stu(ii,:)),2,2) <= tolerances.errTolPos;

        if ~isempty(find(isSol,1))
            numRight = numRight + 1;
        end
    end

    fprintf('\t %d of your %d solutions are correct.\n', numRight, size(q_stu,1));
    
    if size(q_stu,1) > 0
        score(3) = numRight/size(q_stu,1);
    else
        score(3) = 1;
    end
catch ME
    fprintf('Your function returns the following error:\n\n\t');
    disp(ME.message)
    fprintf('\n\nPlease check your code and try again.\n');
end
        
end