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


#include "lpvDS.h"

lpvDS::lpvDS(const char  *path_dims):K_(0),M_(0) {

    /* Declare the number of the components
     * and the dimension  of the state  */
    MatrixXd fMatrix(1,1);fMatrix.setZero();

    if (fileUtils_.is_file_exist(path_dims))
        fMatrix=fileUtils_.readMatrix(path_dims);
    else{
        cout<<"The provided path for Dimensions does not exist:"<< path_dims << endl;
        ERROR();
    }
    if ((fMatrix.rows()!=2)){
        cout<<"Initialization of the Dimensions is wrong."<<endl;
        ERROR();
    }

    K_ = (int)fMatrix.coeff(0,0);
    M_ = (int)fMatrix.coeff(1,0);
    setup_params();
}


lpvDS::lpvDS(const char  *path_dims, const char  *path_prior_,const char  *path_mu_,const char  *path_sigma_, const char  *path_A_):K_(0),M_(0) {

    /* Declare the number of the components
     * and the dimension  of the state  */
    MatrixXd fMatrix(1,1);fMatrix.setZero();

    if ( fileUtils_.is_file_exist(path_dims))
        fMatrix= fileUtils_.readMatrix(path_dims);
    else{
        cout<<"The provided path for Dimensions does not exist:"<< path_dims << endl;
        ERROR();
    }
    if ((fMatrix.rows()!=2)){
        cout<<"Initialization of the Dimensions is wrong."<<endl;
        ERROR();
    }

    K_ = (int)fMatrix.coeff(0,0);
    M_ = (int)fMatrix.coeff(1,0);
    setup_params();
    initialize_gamma(path_prior_, path_mu_, path_sigma_);
    initialize_A(path_A_);
}


lpvDS::lpvDS(int K, int M, const MatrixXd Priors_fMatrix, const MatrixXd Mu_fMatrix, const MatrixXd Sigma_fMatrix, const MatrixXd A_fMatrix ):K_(K),M_(M) {

    /* Given the parameters directly as MatrixXd, initialize all matrices*/
    setup_params();
    initialize_Priors(Priors_fMatrix);
    initialize_Mu(Mu_fMatrix);
    initialize_Sigma(Sigma_fMatrix);
    initialize_A(A_fMatrix);

}


lpvDS::lpvDS(const int K, const int M, const vector<double> Priors_vec, const vector<double> Mu_vec, const vector<double> Sigma_vec, const vector<double> A_vec):K_(K),M_(M){


    /* Given the parameters directly as vector<double>, initialize all matrices*/
    setup_params();
    initialize_Priors_vec(Priors_vec);
    initialize_Mu_vec(Mu_vec);
    initialize_Sigma_vec(Sigma_vec);
    initialize_A_vec(A_vec);
}


lpvDS::~lpvDS(){

}

void lpvDS::ERROR()
{
    cout << "SOMETHING IS WRONG!!!" << endl;
}

void lpvDS::setup_params()
{

    /* Setup matrices */
    A_Matrix_ = new MatrixXd[K_]; for(int s=0; s<K_; s++ ){A_Matrix_[s].resize(M_,M_);}
    Prior_    = new double[K_];
    Mu_       = new VectorXd[K_]; for(int s=0; s<K_; s++ ){	Mu_[s].resize(M_);	}
    Sigma_    = new MatrixXd[K_]; for(int s=0; s<K_; s++ ){	Sigma_[s].resize(M_,M_);	}

    gamma_.resize(K_);
    gamma_.setZero();
    cout << "Initialized an M:" << M_ << " dimensional GMM-based LPV-DS with K: " << K_ << " Components" << endl;
}


/***************************************************/
/* Initialization functions with MatrixXd as input */
/***************************************************/

void lpvDS::initialize_Priors(const MatrixXd fMatrix ){

    cout<<"** Initializing Priors **"<< endl;
    if ((fMatrix.cols()!=K_)||(fMatrix.rows()!=1))
    {
        cout<<"Initialization of Prior is wrong."<<endl;
        cout<<"Number of components is: "<<K_<<endl;
        cout<<"Dimension of states of Prior is: "<<fMatrix.cols()<<endl;
        ERROR();
    }

    for (int i=0; i<K_; i++)
        Prior_[i]=fMatrix(0,i);
}


void lpvDS::initialize_Mu(const MatrixXd fMatrix ){

    cout<<"** Initializing Mu **"<< endl;
    if ((fMatrix.cols()!=K_)||(fMatrix.rows()!=M_)){
        cout<<"Initialization of Mean is wrong."<<endl;
        cout<<"Number of components is: "<<K_<<endl;
        cout<<"Dimension of states is: "<<M_<<endl;
        cout<<"Dimension of states of Mean is: "<<fMatrix.rows()<<"*"<<fMatrix.cols()<<endl;
        ERROR();
    }

    for(int s=0; s<K_; s++ )
        Mu_[s]=fMatrix.col(s);
}

void lpvDS::initialize_Sigma(const MatrixXd fMatrix ){
    cout<<"** Initializing Sigma **"<< endl;
    if ((fMatrix.rows()!=K_*M_)||(fMatrix.cols()!=M_))
    {
        cout<<"Initialization of the covariance matrix is wrong."<<endl;
        cout<<"the covariance matrix : "<<endl;cout<<fMatrix<<endl;
        cout<<"Number of components is: "<< K_ <<endl;
        cout<<"Dimension of states is: "<< M_ <<endl;
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
}

void lpvDS::initialize_A(const MatrixXd fMatrix ){


    cout<<"** Initializing A's' **"<< endl;
    if ((fMatrix.rows()!=K_*M_)||(fMatrix.cols()!=M_))
    {
        cout<<"Initialization of the A matrices is wrong!!"<<endl;
        cout<<"A_k: "<< endl << fMatrix<< endl;
        cout<<"[Proposed Dimensionality] K : "<<K_<<" M:"<<M_<<endl;
        cout<<"[Actual Dimensionality] K : "<< fMatrix.cols()/M_ <<" M:" << fMatrix.rows() << endl;
        ERROR();
    }
    int j = 0;
    for(int s=0; s<K_; s++ ){
        for(int i=0; i<M_; i++ ){
            A_Matrix_[s].row(i)=fMatrix.row(j);
            j++;
        }
    }
}

/*********************************************************/
/* Initialization functions with vector<double> as input */
/*********************************************************/

void lpvDS::initialize_Priors_vec(const vector<double> Priors_vec){
    cout<<"** Initializing Priors **"<< endl;
    if (Priors_vec.size() != K_){
        cout<<"Initialization of Prior is wrong."<<endl;
        cout<<"Number of components is: "<<K_<<endl;
        cout<<"Dimension of states of Prior is: "<<Priors_vec.size()<<endl;
        ERROR();
    }

    for (int k=0; k<K_; k++)
        Prior_[k] = Priors_vec[k];
}


void lpvDS::initialize_Mu_vec(const vector<double> Mu_vec){

    cout<<"** Initializing Mu **"<< endl;
    if (Mu_vec.size() != K_*M_){
        cout<<"Initialization of Mean is wrong."<<endl;
        cout<<"Size of vector should be K ("<<K_ << ")*M("<< M_<< ") =" << K_*M_ <<endl;
        cout<<"Given vector is of size "<< Mu_vec.size() << endl;
        ERROR();
    }


    for(int k=0; k<K_; k++ ){
        VectorXd Mu_k; Mu_k.resize(M_); Mu_k.setZero();
        for (int m = 0; m < M_; m++)
            Mu_k[m] = Mu_vec[k * M_ + m];
        Mu_[k]=Mu_k;
    }

    /* For Debugging */
//    for(int k=0; k<K_; k++ )
//        cout << "Mu["<< k << "]"<< endl << Mu_[k] << endl;
}


void lpvDS::initialize_Sigma_vec(const vector<double> Sigma_vec){

    cout<<"** Initializing Sigma **"<< endl;
    if (Sigma_vec.size() != K_*M_*M_){
        cout<<"Initialization of Sigma is wrong."<<endl;
        cout<<"Size of vector should be K ("<<K_ << ")*M("<< M_<< ")*M(" << M_<< ") =" << K_*M_*M_ <<endl;
        cout<<"Given vector is of size "<< Sigma_vec.size() << endl;
        ERROR();
    }

    for(int k=0; k<K_; k++ ){
        MatrixXd Sigma_k; Sigma_k.resize(M_,M_); Sigma_k.setZero();
        for (int row = 0; row < M_; row++) {
            for (int col = 0; col < M_; col++) {
                int ind = k * M_ * M_ + row * M_ + col;
                Sigma_k(col,row)  = Sigma_vec[ind];
            }
        }
        Sigma_[k] = Sigma_k;
    }

    /* For Debugging */
//    for(int k=0; k<K_; k++ )
//        cout << "Sigma["<< k << "]"<< endl << Sigma_[k] << endl;

}


void lpvDS::initialize_A_vec(const vector<double> A_vec){
    cout<<"** Initializing A **"<< endl;
    if (A_vec.size() != K_*M_*M_){
        cout<<"Initialization of A-matrices is wrong."<<endl;
        cout<<"Size of vector should be K ("<<K_ << ")*M("<< M_<< ")*M(" << M_<< ") =" << K_*M_*M_ <<endl;
        cout<<"Given vector is of size "<< A_vec.size() << endl;
        ERROR();
    }

    for(int k=0; k<K_; k++ ){
        MatrixXd A_k; A_k.resize(M_,M_); A_k.setZero();
        for (int row = 0; row < M_; row++) {
            for (int col = 0; col < M_; col++) {
                int ind = k * M_ * M_ + row * M_ + col;
                A_k(col,row)  = A_vec[ind];
            }
        }
        A_Matrix_[k] = A_k;
    }


    /* For Debugging */
//    for(int k=0; k<K_; k++ )
//        cout << "A["<< k << "]"<< endl << A_Matrix_[k] << endl;
}



/************************************************************/
/* Initialization functions with path to text file as input */
/************************************************************/

void lpvDS::initialize_A(const char  *path_A_){

    /* Initialize A
     * path_A_ is the path of A matrix*/

    MatrixXd fMatrix(1,1);fMatrix.setZero();
    if (fileUtils_.is_file_exist(path_A_))
        fMatrix=fileUtils_.readMatrix(path_A_);
    else{
        cout<<"The provided path does not exist: "<<path_A_<<endl;
        ERROR();
    }

    initialize_A(fMatrix);
}


void lpvDS::initialize_gamma(const char  *path_prior_,const char  *path_mu_,const char  *path_sigma_){

	/* Initialize scheduling/activation function parameters
	 *  path_prior_ is the path of the prior matrix
	 *	path_mu_ is the path of the mean matrix
	 *	path_sigma_ is the path of the covariance matrix */

    /* Initializing Priors*/
    MatrixXd fMatrix;
    if (fileUtils_.is_file_exist(path_prior_))
        fMatrix=fileUtils_.readMatrix(path_prior_);
	else{
		cout<<"The provided path does not exist."<<endl;
		cout<<"path_prior_lpvDS "<<endl;
		cout<<path_prior_<<endl;
		ERROR();
	}
    initialize_Priors(fMatrix);


    /* Initializing Mu*/
	fMatrix.setZero();
    if (fileUtils_.is_file_exist(path_mu_))
        fMatrix=fileUtils_.readMatrix(path_mu_);
	else{
		cout<<"The provided path does not exist."<<endl;
		cout<<"path_mu_lpvDS "<<endl;
		cout<<path_mu_<<endl;
		ERROR();
	}
    initialize_Mu(fMatrix);

    /* Initializing Sigma*/
	fMatrix.resize(1,1);fMatrix.setZero();
    if (fileUtils_.is_file_exist(path_sigma_))
        fMatrix=fileUtils_.readMatrix(path_sigma_);
	else{
		cout<<"The provided path does not exist."<<endl;
		cout<<"path_sigma_lpvDS "<<endl;
		cout<<path_sigma_<<endl;
		ERROR();
	}
    initialize_Sigma( fMatrix );
}

/****************************************/
/*     Actual computation functions     */
/****************************************/

MatrixXd lpvDS::compute_A(VectorXd X){

    /* Calculating the weighted sum of A matrices */

	if ((X.rows()!=M_)){
		cout<<"The dimension of X in compute_A is wrong."<<endl;
		cout<<"Dimension of states is: "<<M_<<endl;
		cout<<"Dimension of X "<<X.rows()<<endl;
		ERROR();
	}

	MatrixXd A; A.resize(M_,M_);A.setZero();
	if (K_>1)
        gamma_= compute_gamma(X);
	else
		gamma_(K_-1)=1;

	for (int i=0;i<K_;i++)
        A = A + A_Matrix_[i]*gamma_(i);

	return A;
}

VectorXd lpvDS::compute_gamma(VectorXd X){
    VectorXd gamma;
    gamma.resize(K_);
    gamma.setZero();

	for (int i=0;i<K_;i++)
        gamma(i)=Prior_[i]*GaussianPDF(X,Mu_[i],Sigma_[i]);

    double sum = gamma.sum();
	if (sum<1e-100){
		for (int i=0;i<K_;i++)
            gamma(i)=1.0/K_;
	}
	else
        gamma = gamma/sum;

    return gamma;
}

VectorXd   lpvDS::compute_f(VectorXd xi, VectorXd att){
    MatrixXd A_matrix; A_matrix.resize(M_,M_); A_matrix.setZero();
    VectorXd xi_dot;     xi_dot.resize(M_);    xi_dot.setZero();

    A_matrix = compute_A(xi);
    xi_dot = A_matrix*(xi - att);

    return xi_dot;
}


double lpvDS::GaussianPDF(VectorXd x, VectorXd Mu, MatrixXd Sigma){

	double p;
	MatrixXd gfDiff;gfDiff.resize(1,M_);
	MatrixXd gfDiff_T;gfDiff_T.resize(M_,1);
	MatrixXd SigmaIIInv;SigmaIIInv.resize(M_,M_);
	double detSigmaII=0;
	MatrixXd gfDiffp;gfDiffp.resize(1,1);gfDiffp.setZero();

	detSigmaII=Sigma.determinant();
	SigmaIIInv=Sigma.inverse();

    if (detSigmaII<0)
        detSigmaII=0;

	gfDiff=(x - Mu).transpose();
	gfDiff_T=x - Mu;
	gfDiffp =gfDiff*SigmaIIInv* gfDiff_T;
	gfDiffp(0,0)=fabs(0.5*gfDiffp(0,0));
	p = exp(-gfDiffp(0,0)) / sqrt(pow(2.0*PI_, M_)*( detSigmaII +1e-50));

    return p;
}


