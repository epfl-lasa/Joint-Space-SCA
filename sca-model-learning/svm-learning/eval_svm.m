function [ACC, F, CONF, Prec, TNR, TPR] = eval_svm(t, est)

cor = t == est;
dif = t ~= est;

pos = t == 0;
neg = t == 1;

P = nnz(pos);
N = nnz(neg);
%%%%%%%%%%%%%%%%%Confusion Matrix
%
%%%%%%%%%%%%%%        ESTIMATED
%%%%%%%%%%%%%% | Positive | Negative|
%------------------------------------
%%%%%|Positive |    TP    |   FN    |
%REAL|-------- |--------------------|
%%%%%|Negative |    FP    |   TN    |
%-----------------------------------
%
TP = nnz(cor & pos);  FN = nnz(dif & pos); 
FP = nnz(dif & neg);  TN = nnz(cor & neg); 

CONF.TP = find(cor & pos);
CONF.FN = find(dif & pos);
CONF.FP = find(dif & neg);
CONF.TN = find(cor & neg);

CONF.matrix = [TP FN; FP TN];
CONF.matrix_norm_rel = [TP/P FN/P; FP/N TN/N];
CONF.matrix_norm = [TP/(P+N) FN/(P+N); FP/(P+N) TN/(P+N)];

ACC = (TP + TN)/(P + N);
Prec = TP/(TP+FP);
TPR = TP/(TP+FN);
F = 2*Prec*TPR/(Prec+TPR);
TNR = TN / (TN+FP);
end