clc
clear all
k = 0;
CONF = cell(1);
%for k = 1:1:10
test_est = load("cpsp_pred/pred_"+string(k)+".txt");
test_est(test_est>0) = 1;
test_est(test_est<0) = 0;

size_test = "100k_end";
test_dataset = "libsvm_data/datasets/"+string(k)+"_icub_dataset_libsvm_" + size_test + ".txt";
test_truth = readmatrix(test_dataset,'FileType','text');
test_truth = test_truth(:,1);
k = k+1
AC(k), F(k), CONF{k}, PREC(k), TNR(k), TPR(k)] = eval_svm(test_truth, test_est);
TP(k) = CONF{k}.matrix_norm(1,1);
TN(k) = CONF{k}.matrix_norm(2,2);
FP(k) = CONF{k}.matrix_norm(2,1);
FN(k) = CONF{k}.matrix_norm(1,2);
%end