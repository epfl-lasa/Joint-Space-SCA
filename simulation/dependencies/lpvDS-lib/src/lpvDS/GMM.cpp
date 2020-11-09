/*
 * Copyright (C) 2018 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
 * Author:  Sina Mirrazavi and Nadia Figueroa
 * email:   {sina.mirrazavi,nadia.figueroafernandez}@epfl.ch
 * website: lasa.epfl.ch
 *
 * This work was supported by the EU project Cogimon H2020-ICT-23-2014.
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include "GMM.h"

void GMM::initialize(int Num_Com,int Num_state)
{

    /* Declare the number of the components
     * and the dimension  of the state  */

    K_      =   Num_Com;
    M_      =   Num_state;

    Prior_  =   new double[K_];
    Mu_     =   new VectorXd[K_];	for(int s=0; s<K_; s++ ){	Mu_[s].resize(M_);	}
    Sigma_  =   new MatrixXd[K_];	for(int s=0; s<K_; s++ ){	Sigma_[s].resize(M_,M_);	}


}
void GMM::initialize_GMM(const char  *path_prior_,const char  *path_mu_,const char  *path_sigma_, const char *path_threshold){

    /* Initialize scheduling parameters
     *  path_prior_ is the path of the prior matrix
     *	path_mu_ is the path of the mean matrix
     *	path_sigma_ is the path of the covariance matrix */

    cout<<"Initializing the model of workspace"<<endl;

    MatrixXd fMatrix;
    if (fileUtils_.is_file_exist(path_prior_))
    {
        fMatrix=fileUtils_.readMatrix(path_prior_);
    }
    else
    {
        cout<<"The provided path does not exist."<<endl;
        cout<<"path_prior_GMM "<<endl;
        cout<<path_prior_<<endl;
        ERROR();
    }

    cout<<"fMatrix "<<endl;cout<<fMatrix<<endl;

    if ((fMatrix.cols()!=K_)||(fMatrix.rows()!=1))
    {
        cout<<"Initialization of Prior is wrong."<<endl;
        cout<<"Dimension of states is: "<<K_<<endl;
        cout<<"Dimension of states of Prior is: "<<fMatrix.cols()<<endl;
        ERROR();
    }

    cout<<"Prior"<<endl;
    for (int i=0; i<K_; i++)
    {
        Prior_[i]=fMatrix(0,i);
    }
    for (int i=0; i<K_; i++)
    {
        cout<<Prior_[i]<<endl;
    }

    fMatrix.setZero();
    if (fileUtils_.is_file_exist(path_mu_))
    {
        fMatrix=fileUtils_.readMatrix(path_mu_);
    }
    else
    {
        cout<<"The provided path does not exist."<<endl;
        cout<<"path_mu_GMM "<<endl;
        cout<<path_mu_<<endl;
        ERROR();
    }


    if ((fMatrix.cols()!=K_)||(fMatrix.rows()!=M_))
    {
        cout<<"Initialization of Mean is wrong."<<endl;
        cout<<"Number of components is: "<<K_<<endl;
        cout<<"Dimension of states is: "<<M_<<endl;
        cout<<"Dimension of states of Mean is: "<<fMatrix.rows()<<"*"<<fMatrix.cols()<<endl;
        ERROR();
    }

    for(int s=0; s<K_; s++ )
    {
        Mu_[s]=fMatrix.col(s);

    }

    for(int s=0; s<K_; s++ )
    {
        cout<<"Mu["<<s<<"]."<<endl;
        cout<<Mu_[s]<<endl;

    }

    fMatrix.resize(1,1);fMatrix.setZero();


    if (fileUtils_.is_file_exist(path_sigma_))
    {
        fMatrix=fileUtils_.readMatrix(path_sigma_);
    }
    else
    {
        cout<<"The provided path does not exist."<<endl;
        cout<<"path_sigma_GMM "<<endl;
        cout<<path_sigma_<<endl;
        ERROR();
    }


    if ((fMatrix.rows()!=K_*M_)||(fMatrix.cols()!=M_))
    {
        cout<<"Initialization of the covariance matrix is wrong."<<endl;
        cout<<"the covariance matrix : "<<endl;cout<<fMatrix<<endl;
        cout<<"Number of components is: "<<K_<<endl;
        cout<<"Dimension of states is: "<<M_<<endl;
        cout<<"Dimension of states of the covariance matrix is: "<<fMatrix.rows()<<"*"<<fMatrix.cols()<<endl;
        ERROR();
    }

    int s=0;

    for(int i=0; i<K_; i++ ){
        for(int j=0; j<M_; j++ ){
            Sigma_[i].row(j)=fMatrix.row(s);
            s++;
        }
    }

    for(int i=0; i<K_; i++ ){
        cout<<"Sigma["<<i<<"]."<<endl;
        cout<<Sigma_[i]<<endl;
    }
    fMatrix.resize(1,1);fMatrix.setZero();


    if (fileUtils_.is_file_exist(path_threshold))
    {
        fMatrix=fileUtils_.readMatrix(path_threshold);
    }
    else
    {
        cout<<"The provided path does not exist."<<endl;
        cout<<"path_threshold "<<endl;
        cout<<path_threshold<<endl;
        ERROR();
    }

    threshold=fMatrix(0,0);

    cout<<"The workspace threshold is "<<threshold<<endl;

}
double GMM::PDF(VectorXd X)
{
    VectorXd Theta;Theta.resize(K_);Theta.setZero();

    for (int i=0;i<K_;i++)
    {
        Theta(i)=Prior_[i]*GaussianPDF(X,Mu_[i],Sigma_[i]);
    }
    double sum=Theta.sum();


    return sum-threshold;
}


void GMM::ERROR()
{
    cout << "SOMETHING IS WRONG!!!" << endl;
}
double GMM::GaussianPDF(VectorXd x,VectorXd Mu,MatrixXd Sigma)
{

    double p;
    MatrixXd gfDiff;gfDiff.resize(1,M_);
    MatrixXd gfDiff_T;gfDiff_T.resize(M_,1);
    MatrixXd SigmaIIInv;SigmaIIInv.resize(M_,M_);
    double detSigmaII=0;
    MatrixXd gfDiffp;gfDiffp.resize(1,1);gfDiffp.setZero();

    detSigmaII=Sigma.determinant();
    SigmaIIInv=Sigma.inverse();
    if (detSigmaII<0)
    {
        detSigmaII=0;
    }
    gfDiff=(x - Mu).transpose();
    gfDiff_T=x - Mu;
    gfDiffp =gfDiff*SigmaIIInv* gfDiff_T;
    gfDiffp(0,0)=fabs(0.5*gfDiffp(0,0));

    p = exp(-gfDiffp(0,0)) / sqrt(pow(2.0*PI, M_)*( detSigmaII +1e-50));
    return p;
}
