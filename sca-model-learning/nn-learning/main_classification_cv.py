# %% Load collision dataset
import time
import torch
import numpy as np
from sklearn.datasets import load_svmlight_file
from sklearn.model_selection import train_test_split

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
idx = 10 #run script separately for each submodel 1-10
path = '../datasets/'
fname_train = str(idx)+'_icub_dataset_libsvm_900k_start.txt'
fname_test = str(idx)+'_icub_dataset_libsvm_100k_end.txt'

data_train = load_svmlight_file(path+fname_train)
x_tens_train = torch.Tensor(data_train[0].toarray());
y_tens_train = torch.Tensor(data_train[1]);
dim = x_tens_train.size(1);
sz = x_tens_train.size(0);

data_test = load_svmlight_file(path+fname_test)
x_tens_test = torch.Tensor(data_test[0].toarray());
y_tens_test = torch.Tensor(data_test[1]);

all_data = np.concatenate((x_tens_train,x_tens_test))
all_labels = np.concatenate((y_tens_train,y_tens_test))

for cv in range(1): #increase the number of iterations to perform crossvalidation
    X_train, X_test, y_train, y_test = train_test_split(all_data, all_labels, test_size=0.05555)
    x_tens_train = torch.Tensor(X_train);
    y_tens_train = torch.Tensor(y_train);
    x_tens_test = torch.Tensor(X_test);
    y_tens_test = torch.Tensor(y_test);
    
    # %% NN definition
    import torch.nn as nn
    import torch.nn.functional as F
    
    class Net(nn.Module):
        def __init__(self):
            super(Net, self).__init__()
            self.l1 = nn.Linear(dim, 50)
            self.l2 = nn.Linear(50, 30)
            self.l3 = nn.Linear(30, 10)
            self.l4 = nn.Linear(10, 2)
            self.act = nn.Tanh()
            #self.lsm = nn.LogSoftmax()
            #self.sm = nn.Softmax(dim=-1)
        def forward(self, x):
            x = self.l1(x)
            x = self.act(x)
            x = self.l2(x)
            x = self.act(x)
            x = self.l3(x)
            x = self.act(x)
            x = self.l4(x)
            #x = self.lsm(x)
            return x
    
    net = Net()
    
    print(net)
    params = list(net.parameters())
    print(len(params))
    print(params[0].size())
    from scipy.io import loadmat, savemat
    # nn_ml = loadmat('data/nn'+str(idx)+'.mat')
    
    # net.state_dict()['l1.weight'] *= 0
    # net.state_dict()['l1.weight'] += torch.from_numpy(nn_ml['w1'])
    
    # net.state_dict()['l2.weight'] *= 0
    # net.state_dict()['l2.weight'] += torch.from_numpy(nn_ml['w2'])
    
    # net.state_dict()['l3.weight'] *= 0
    # net.state_dict()['l3.weight'] += torch.from_numpy(nn_ml['w3'])
    
    # net.state_dict()['l4.weight'] *= 0
    # net.state_dict()['l4.weight'] += torch.from_numpy(nn_ml['w4'])
    
    
    # net.state_dict()['l1.bias'] *= 0
    # net.state_dict()['l1.bias'] += torch.from_numpy(nn_ml['b1']).transpose(0,1).squeeze(0)
    
    # net.state_dict()['l2.bias'] *= 0
    # net.state_dict()['l2.bias'] += torch.from_numpy(nn_ml['b2']).transpose(0,1).squeeze(0)
    
    # net.state_dict()['l3.bias'] *= 0
    # net.state_dict()['l3.bias'] += torch.from_numpy(nn_ml['b3']).transpose(0,1).squeeze(0)
    
    # net.state_dict()['l4.bias'] *= 0
    # net.state_dict()['l4.bias'] += torch.from_numpy(nn_ml['b4']).transpose(0,1).squeeze(0)
    
    # %% Loss fcn
    import torch.optim as optim
    
    criterion = nn.CrossEntropyLoss()
    #criterion = nn.NLLLoss()
    
    optimizer = optim.Rprop(net.parameters())
    
    #optimizer = optim.Adadelta(net.parameters())
    #optimizer = optim.SGD(net.parameters(),lr = 0.1, momentum = 0.9)
    # inputs = x_tens_train[0]
    # labels = y_tens_train[0]
    # outputs = net(inputs)
    # loss = criterion(outputs, labels)
    # %% Training
    net.to(device)
    x_gpu = x_tens_train.to(device)
    y_gpu = y_tens_train.to(device = device, dtype=torch.int64)
    t = time.time()
    for epoch in range(1000):  # loop over the dataset multiple times, 5000 for the best accuracy
        running_loss = 0.0
    
        #print(labels)
        # zero the parameter gradients
        inputs, labels = x_gpu, y_gpu
        optimizer.zero_grad()
        
        # forward + backward + optimize
        outputs = net(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        # print statistics
        running_loss += loss.item()
        
        print('Epoch %d, Loss: %.3f' %
              (epoch + 1, running_loss))
        running_loss = 0.0
    
    print('Finished Training')
    elapsed = time.time() - t
    print('Time:%5.1f seconds' % elapsed)
    # %% save model
    #PATH = './cifar_net.pth'
    #torch.save(net.state_dict(), PATH)
    
    # %% verification
    postures = x_tens_test.to(device)
    labels = y_tens_test.to(device = device, dtype=torch.int64)
    net.to('cpu')
    postures = x_tens_test
    labels = y_tens_test.to(dtype=torch.int64)
    with torch.no_grad():
        outputs = net(postures)
        _, predicted = torch.max(outputs.data,1)
        total = labels.size(0)
        correct = (predicted == labels).sum().item()
    print('Accuracy of the network on the test dataset: %.5f %%' % (
        100 * correct / total))
    
    # t = time.time()
    # n_b = 1000;
    # with torch.no_grad():
    #     for i in range(n_b):
    #         outputs = net(postures[i])
    # print('Time for single prediction :%.10f seconds' % ((time.time() - t)/n_b))
    
    # %% num weights
    total_sz = 0;
    
    savedict = dict();
    for key in net.state_dict().keys():
        total_sz += np.prod(net.state_dict()[key].size())
        savedict[key.replace(".", "_")]=net.state_dict()[key].cpu().numpy()
    #savemat('nn_py'+str(idx)+'_'+str(cv)+'.mat',savedict)
    savemat('nn_py'+str(idx)+'.mat',savedict)

    # Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
    #traced_script_module = torch.jit.trace(net, torch.ones(1,x_tens_train.size()[1]))
    #traced_script_module.save("data/"+str(idx)+".pt")
    

# %% jac
# def get_jacobian(net, x, noutputs):
#     x = x.squeeze()
#     n = x.size()[0]
#     x = x.repeat(noutputs, 1)
#     x.requires_grad_(True)
#     y = net(x)
#     y.backward(torch.eye(noutputs))
#     return y[0], x.grad.data

# t = time.time()
# n_b = 1;
# for i in range(n_b):
#     a, b = get_jacobian(net,postures[i],2)
# print('Time for single prediction :%.10f seconds' % ((time.time() - t)/n_b))
# xz = torch.tensor(np.zeros([1,14])).to(dtype=torch.float)
# a,b = get_jacobian(net,xz,2)
# print(a)
# print(b)
# print(xz)