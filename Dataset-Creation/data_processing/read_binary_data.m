%% read binary data
%clc
%clear all

function data = read_binary_data(fname)
    %fname = "../dataset/data.bin";
    fileID = fopen(fname,'r');
    n_trials = fread(fileID,1,'int');
    l_size = fread(fileID,1,'int');
    data = zeros(n_trials,l_size);
    for i = 1:1:n_trials
        data(i,:) = fread(fileID,l_size,'float');
    %     if data(i,1) == 0
    %        data(i,1) = -1; 
    %     end
    end
    fclose(fileID);
end


