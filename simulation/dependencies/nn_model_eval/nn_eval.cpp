#include "nn_eval.h"
using namespace std;
using namespace Eigen;

nn_state nn_model_state_load(std::string fname){
  nn_state output;
  ifstream file (fname, ios::in|ios::binary);
  if(file) {
    vector<MatrixXd> D;
    while(true){  
      int ndim; 
      file.read((char*)&ndim,sizeof(ndim));
      //cout << ndim << endl;
      vector<int> sz(ndim);
      int total_elem = 1;
      for(int i = 0;i<ndim; i++)
      {
        file.read((char*)&sz[i],sizeof(sz[i]));
        total_elem = total_elem * sz[i];
        //cout << sz[i] << endl;
      }
      MatrixXd data(sz[0],sz[1]);
      for(int i = 0;i<total_elem; i++)
      {
        file.read((char*)&data(i),sizeof(data(i)));
      }
      //cout << data << endl;
      D.push_back(data);
      if(file.eof())break;
    }
    //cout << D.size() << endl;
    output.w1 = D[0];
    output.w2 = D[1];
    output.w3 = D[2];
    output.w4 = D[3];
    output.b1 = D[4];
    output.b2 = D[5];
    output.b3 = D[6];
    output.b4 = D[7];
    file.close();
    cout << fname <<" is loaded!" << endl;
    }
  else {
    cout << fname <<" is NOT loaded!" << endl;
  }
  return output;
}

void calcGamma_nn_manual(VectorXd x, nn_state &m, double &Gamma, VectorXd &GammaGrad){

  MatrixXd z1 = m.w1*x+m.b1;
  MatrixXd h1 = tanh(z1.array());
  //MatrixXd h1m = MatrixXd::Zero(h1.size(),h1.size());
  //h1m.diagonal() << h1;

  MatrixXd z2 = m.w2*h1+m.b2;
  MatrixXd h2 = tanh(z2.array());
  //MatrixXd h2m = MatrixXd::Zero(h2.size(),h2.size());
  //h2m.diagonal() << h2;

  MatrixXd z3 = m.w3*h2+m.b3;
  MatrixXd h3 = tanh(z3.array());
  // MatrixXd h3m = MatrixXd::Zero(h3.size(),h3.size());
  // h3m.diagonal() << h3;
  MatrixXd z4 = m.w4*h3+m.b4;
  //cout << h1.size() << endl;
  MatrixXd s1 = MatrixXd::Zero(h1.size(),h1.size());
  MatrixXd s2 = MatrixXd::Zero(h2.size(),h2.size());
  MatrixXd s3 = MatrixXd::Zero(h3.size(),h3.size());
  s1.diagonal() << 1-h1.array().square();
  s2.diagonal() << 1-h2.array().square();
  s3.diagonal() << 1-h3.array().square();


  MatrixXd dz4 = m.w4*s3*m.w3*s2*m.w2*s1*m.w1;

  Gamma = z4(0)-z4(1);
  GammaGrad = dz4.row(0) - dz4.row(1);
  //cout << Gamma << endl << GammaGrad.transpose() << endl;
  // cout << h1.transpose() <<endl <<endl;
  // cout << h2.transpose() <<endl<<endl;
  // cout << h3.transpose() <<endl<<endl;
  // cout << z4.transpose() <<endl<<endl;
  // cout << dz4.transpose() <<endl<<endl; 
}
