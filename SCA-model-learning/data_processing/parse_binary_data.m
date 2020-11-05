clc
clear all

%for part_idx = 1:1:10
    
part_idx = 1; %1-10 body pair index
part_idx
%dataset_size = 250000; mode = "start";
%dataset_size = 150000; mode = "end";

%dataset_size = 900000; mode = "start";
%dataset_size = 100000; mode = "end";

%dataset_size = 850000; mode = "start"; //for 900k 1_models
dataset_size = 50000; mode = "end";
%mode = "start"; %taking values from the beginning of data array (training)
%mode = "end"; %taking values from the end (testing)
flist = dir("../dataset_fcl/dataset_parts/data/");
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
    if flist(i).part == part_idx & flist(i).ts > 1599036920
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

fname = "libsvm_data/datasets/"+part_idx+"_icub_dataset_libsvm_"+ string(dataset_size/1000) +"k_"+mode+".txt";

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
%end