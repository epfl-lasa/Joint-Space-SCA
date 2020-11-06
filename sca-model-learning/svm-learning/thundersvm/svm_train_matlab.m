function svm_train_matlab(a)
    str = {'thundersvm-train'};
	str2 = [str a];
    if isunix
        if  not(libisloaded('libthundersvm'))
            loadlibrary('thundersvm/libthundersvm.so', 'thundersvm/include/thundersvm/svm_interface_api.h')
        end
        calllib('libthundersvm', 'thundersvm_train', length(str2), str2)
    else
        disp 'OS not supported!'
    end
