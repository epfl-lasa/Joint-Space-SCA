function write_binary_data(fname, data)
    fid  = fopen(fname, 'w');
    if fid == - 1
      error('Cannot open file for writing');
    end
    for i = 1:1:size(data,2)
        fwrite(fid, ndims(data{i}), 'int');
        fwrite(fid, size(data{i}), 'int');
        fwrite(fid, data{i}, 'double');
    end
    fclose(fid);
end




