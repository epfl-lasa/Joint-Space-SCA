function svm_train_matlab(a)
    str = {'thundersvm-predict'};
	str2 = [str a];
    if isunix
        if  not(libisloaded('libthundersvm'))
            loadlibrary('thundersvm/libthundersvm.so', 'thundersvm/include/thundersvm/svm_interface_api.h')
        end
        calllib('libthundersvm', 'thundersvm_predict', length(str2), str2)
    else
        disp 'OS not supported!'
    end
