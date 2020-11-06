for idx = 1:1:10
    load(['nn_py' num2str(idx) '.mat'])
    w1 = l1_weight;
    w2 = l2_weight;
    w3 = l3_weight;
    w4 = l4_weight;
    b1 = l1_bias';
    b2 = l2_bias';
    b3 = l3_bias';
    b4 = l4_bias';
    write_binary_data(['../models/nn_model_' num2str(idx) '.dat'],{w1,w2,w3,w4,b1,b2,b3,b4});
end