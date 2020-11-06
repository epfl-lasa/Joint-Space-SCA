clc
clear all
addpath('thundersvm');
%% libSVM options:
% -s svm_type : set type of SVM (default 0)
% 	0 -- C-SVC
% 	1 -- nu-SVC
% 	2 -- one-class SVM
% 	3 -- epsilon-SVR
% 	4 -- nu-SVR
% -t kernel_type : set type of kernel function (default 2)
% 	0 -- linear: u'*v
% 	1 -- polynomial: (gamma*u'*v + coef0)^degree
% 	2 -- radial basis function: exp(-gamma*|u-v|^2)
% 	3 -- sigmoid: tanh(gamma*u'*v + coef0)
% -d degree : set degree in kernel function (default 3)
% -g gamma : set gamma in kernel function (default 1/num_features)
% -r coef0 : set coef0 in kernel function (default 0)
% -c cost : set the parameter C of C-SVC, epsilon-SVR, and nu-SVR (default 1)
% -n nu : set the parameter nu of nu-SVC, one-class SVM, and nu-SVR (default 0.5)
% -p epsilon : set the epsilon in loss function of epsilon-SVR (default 0.1)
% -m cachesize : set cache memory size in MB (default 100)
% -e epsilon : set tolerance of termination criterion (default 0.001)
% -h shrinking: whether to use the shrinking heuristics, 0 or 1 (default 1)
% -b probability_estimates: whether to train a SVC or SVR model for probability estimates, 0 or 1 (default 0)
% -wi weight: set the parameter C of class i to weight*C, for C-SVC (default 1)
% 
% The k in the -g option means the number of attributes in the input data.

%% hyperparameters gridsearch parameters
k = 9;
size_train = "250k_start";
size_test = "150k_end";
idx = string(k);
modelname = "../models/"+idx+"_icub_dataset_libsvm_" + size_train + ".model";
train_dataset = "../datasets/"+idx+"_icub_dataset_libsvm_" + size_train + ".txt";
test_dataset = "../datasets/"+idx+"_icub_dataset_libsvm_" + size_test + ".txt";
test_result = "../datasets/tests/"+idx+"_icub_dataset_libsvm_" + size_test + ".out";
%c_grid = logspace(-1,4,10);
%g_grid = logspace(-2,10);
c_grid = linspace(1,1000,5);
g_grid = linspace(0.1,1,5);

test_truth = readmatrix(test_dataset,'FileType','text');
test_truth = test_truth(:,1);
%diaryname = "libsvm_data/crossvalidation_diary/diary_"+idx+"_(grid_1).txt";
%% main parameters selection cycle
%delete(diaryname)
%diary(diaryname)
for i = 1:1:length(c_grid)
    for j = 1:1:length(g_grid)
        i;
        j;
        c = c_grid(i);
        g = g_grid(j);
        fprintf('\nCV grid #%i,%i, C = %f, gamma = %f \n',i,j,c,g);
        %n = ["-c", num2str(c), "-g", num2str(g),"-e", "0.001","-v", "3", train_dataset, modelname];
        n = ["-c", num2str(c), "-g", num2str(g),"-e", "0.001", train_dataset, modelname];
        svm_train_matlab(cellstr(n));
        
        %hold-out validation
        n = [test_dataset, modelname, test_result];
        svm_predict_matlab(cellstr(n));
        test_est = readmatrix(test_result,'FileType','text');
        [acc(i,j), f(i,j), conf{i,j}, prec(i,j), recall(i,j),~] = eval_svm(test_truth, test_est);
    end
end
%diary off
%save("lgrid_"+idx+"a")
%end

%[c_grid, g_grid, acc] = parse_cv_diary(diaryname);
figure(1)
n_steps = 50;
contourf(c_grid, g_grid, acc', n_steps, 'LineColor','None');
xlabel('C');
ylabel('gamma');
title('Accuracy');
%% code used to train SVM with GPU
%0 - 100, 0.1
%1 - 2000, 0.5
%2,6 - 400, 0.4
%3 - 1000, 0.4
%5 - 200,0.4
%4, 7 - 100, 1.5
%8 - 5000, 0.5
%9,10 - 0.5 500
c_array = [2000 400 1000 100 200 400 100 5000 500 500];
c_array = [1000 400 1000 100 200 400 100 1000 500 500];
g_array = [0.5 0.4 0.4 0.5 0.4 0.4 0.5 0.5 0.5 0.5];
CV = 1;
for k = 1:1:10
    %res = cell(1,CV);
    for d = 1:1:CV
        k = 10;
        c = c_array(k);
        g = g_array(k);
        size_train = "250k_start";
        size_test = "150k_end";
        %modelname = "libsvm_data/models/"+string(k)+"_icub_dataset_libsvm_" + size_train + ".model";
        modelname = "../models/"+string(k)+"_svm_250k" + ".model";
        train_dataset = "../datasets/"+string(k)+"_icub_dataset_libsvm_" + size_train + ".txt";
        test_dataset = "../datasets/"+string(k)+"_icub_dataset_libsvm_" + size_test + ".txt";
        test_result = "../datasets/tests/"+string(k)+"_icub_dataset_libsvm_" + size_test + ".out";
        %tic
        %routine to perform crossvalidation, only necessary for
        %verification of the training
        %[train_dataset, test_dataset] = crossvalidation({train_dataset, test_dataset}, 150000);
        %toc
        n = ["-c", num2str(c), "-g", num2str(g), train_dataset, modelname];
        tic
        svm_train_matlab(cellstr(n));
        time_training = toc;
        n = [test_dataset, modelname, test_result];
        svm_predict_matlab(cellstr(n));
        fprintf('\n Time for training: %f seconds \n', time_training);
        test_truth = readmatrix(test_dataset,'FileType','text');
        test_truth = test_truth(:,1);
        test_est = readmatrix(test_result,'FileType','text');
        [AC, F, CONF, PREC, TNR, TPR] = eval_svm(test_truth, test_est);
        %fprintf(' Trained SVM parameters: \n Accuracy  = %f \n F-measure = %f \n Precision = %f \n Recall    = %f\n\n',AC,F,PREC,REC);
        fprintf(' Trained SVM parameters: \n Accuracy = %f \n TPR = %f \n TNR = %f\n\n',AC,TPR,TNR);
        res{d}.AC = AC;
        res{d}.F = F;
        res{d}.CONF = CONF;
        res{d}.TNR = TNR;
        res{d}.TPR = TPR;
        res{d}.T = time_training;
    end
    %save(sprintf('res_%i.mat',k));
end