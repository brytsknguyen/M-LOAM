clear all;
close all;

tic

path        = matlab.desktop.editor.getActiveFilename;
this_dir    = path(1: end - length(mfilename) - 2);
cd(this_dir);

tests       = dir([this_dir 'result_*']);
tests_count = length(tests);

EXP_NAME    = cell(tests_count, 1);
P_ATE       = cell(tests_count, 1);
R_ATE       = cell(tests_count, 1);

t_shift     = 0.0;

myCluster   = parcluster('local');

fprintf('Number of tests: %d. Number of Workers: %d\n\n',...
         length(tests), myCluster.NumWorkers);

workers_needed = min(6, length(tests));
     
if myCluster.NumWorkers < workers_needed
    delete(gcp);
    parpool(workers_needed);
end

% for n=1:tests_count
parfor n=1:tests_count
    
    [P_ate, rot_rmse] = check_viral_cal_err_parall(n, tests(n));

    EXP_NAME(n) = {tests(n).name(8:end)};
    P_ATE(n)    = {P_ate};
    R_ATE(n)    = {rot_rmse};
    
%     pause;
end

% Number of statistics
M = size(P_ATE{1}, 2);
P_ATE_ = zeros(tests_count, M);

RESULT = cell(tests_count, 2*M+1);

for n=1:tests_count
    RESULT(n, 1) = EXP_NAME(n);
    for m=1:M
        P_ATE_ = P_ATE{n, 1};
        RESULT(n, m+1) = {P_ATE_(m)};
    end
    for m=1:M
        R_ATE_ = R_ATE{n, 1};
        RESULT(n, M + 1 + m) = {R_ATE_(m)};
    end
end

RESULT

save('tests.mat', 'RESULT');

toc