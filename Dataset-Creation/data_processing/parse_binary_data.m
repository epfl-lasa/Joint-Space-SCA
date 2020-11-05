clc
clear all
% This function requires to specify the size of the desired dataset, and
% what side take tha data from (start-end).
% libsvm (and thundersvm) require separate .txt file for each dataset
% (train, test), that's why we generate those .txt in such manner instead
% of putting all the data in single .txt
% In .bin we have class labels unordered, here we sort them and take sz/2
% of positive class and sz/2 from negative from beginning or end of the
% collected dataset. 
% mode = "start" - taking values from the beginning of data array (training)
% mode = "end" - taking values from the end (testing)
% generally you need to launch this function 4 times, uncommenting one by one first
% four lines of the cycle to generate 250k & 150k datasets for svm
% train/test respectively, and 900k/100k for NN's.
% beware that to run this code you need to first generate 10x100k .bin
% datasets for each submodel (i.e. 10 x 10x100k in total).
for part_idx = 1:1:10    
disp(part_idx)
% these lines should be uncommented one by one within four runs
%dataset_size = 250000; mode = "start"; %svm training data
%dataset_size = 150000; mode = "end"; %svm testing data
dataset_size = 900000; mode = "start"; %nn training data
%dataset_size = 100000; mode = "end"; %nn testing data
%%%%%%%%%%%%%%%

flist = dir("../fcl-sampling/data/"); %specify directory wioth .bin files
fnames = [];
data = [];
%check all files in folder and push all "data_#part_idx_%" to "data"
for i = 1:1:length(flist)
    fname = flist(i).name;
    if length(fname)>4 && fname(end-3:end) == ".bin" && fname(1:5) == "data_"
        if(fname(7) == '_')
            flist(i).part = str2double(fname(6));
        else
            flist(i).part = str2double(fname(6:7));
        end
        flist(i).ts = str2double(fname(end-13:end-4));
    else
        flist(i).part = [];
        flist(i).ts = [];
    end    
    if flist(i).part == part_idx %& flist(i).ts > 1599036920
        tmp = read_binary_data(flist(i).folder+"/"+flist(i).name);
        data = [data; tmp];
    end
end
%%
%data = data(randperm(length(data)),:);
idx_active = find(data(1,2:end))+1; %find active joints (fix for 1st column)
idx_neg = find(data(:,1)==0);
idx_pos = find(data(:,1)==1);
d0 = data(idx_neg,:);
d1 = data(idx_pos,:);
half_size = dataset_size / 2;

if mode == "start"
    X = [d0(1:half_size,idx_active); d1(1:half_size,idx_active)];
    y = [d0(1:half_size,1); d1(1:half_size,1)];
elseif mode == "end"
    X = [d0(end-half_size+1:end,idx_active); d1(end-half_size+1:end,idx_active)];
    y = [d0(end-half_size+1:end,1); d1(end-half_size+1:end,1)];
end

fname = "../../SCA-model-learning/datasets/"+part_idx+"_icub_dataset_libsvm_"+ string(dataset_size/1000) +"k_"+mode+".txt";

tic
f_id = fopen(fname,'w');
format_str = '%i';

for i = 1:1:size(X,2)
    format_str = [format_str, ' ', num2str(i),':%f'];
end
format_str = [format_str, '\n'];

for i = 1:1:dataset_size
    vec = X(i,:);
    class = y(i) == 1;
    fprintf(f_id,format_str,class, vec);
end

toc
fclose(f_id);
end