function [name_train, name_test] = crossvalidation(name, n_test)
%k = 1;
%size_train = "250k_start";
%size_test = "150k_end";
%name = cell(0,2);
%n_test = 150000;
%name{1} = "libsvm_data/datasets/"+string(k)+"_icub_dataset_libsvm_" + size_train + ".txt";
%name{2} = "libsvm_data/datasets/"+string(k)+"_icub_dataset_libsvm_" + size_test + ".txt";
data = cell(0,1);
n = zeros(1,length(name));
for i = 1:1:length(name)
    fid=fopen(name{i});
    tline = fgetl(fid);
    while ischar(tline)
        data{end+1,1} = tline;
        tline = fgetl(fid);
        n(i)=n(i)+1;
    end
    fclose(fid);
end

cv = cvpartition(length(data),'holdout',n_test);
data_train = data(cv.training);
data_test = data(cv.test);
name_train = "tmp_train.txt";
fid=fopen(name_train,'w');
fprintf(fid,'%s\n', data_train{:});
fclose(fid);

name_test = "tmp_test.txt";
fid=fopen(name_test,'w');
fprintf(fid,'%s\n', data_test{:});
fclose(fid);
% c = 0;
% for i = 1:1:length(data_test)
%     c = c+str2num(data_test{i}(1));
% end
% d = 0;
% for i = 1:1:length(data_train)
%     d = d+str2num(data_train{i}(1));
% end