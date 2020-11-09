#include "Quaternions.h"

void dc2ang(double dircos[3][3],
    double *a1,
    double *a2,
    double *a3)
{
    double th1,th2,th3,temp[10];

    if (((fabs(dircos[0][2])-1.) >= -1e-15)  ) {
        th1 = atan2(dircos[2][1],dircos[1][1]);
        if ((dircos[0][2] > 0.)  ) {
            temp[0] = 1.5707963267949;
        } else {
            temp[0] = -1.5707963267949;
        }
        th2 = temp[0];
        th3 = 0.;
    } else {
        th1 = atan2(-dircos[1][2],dircos[2][2]);
        th2 = asin(dircos[0][2]);
        th3 = atan2(-dircos[0][1],dircos[0][0]);
    }
    *a1 = th1;
    *a2 = th2;
    *a3 = th3;
}

void dc2quat(double dircos[3][3],
    double *e1,
    double *e2,
    double *e3,
    double *e4)
{
    double tmp,tmp1,tmp2,tmp3,tmp4,temp[10];

    tmp = (dircos[0][0]+(dircos[1][1]+dircos[2][2]));
    if (((tmp >= dircos[0][0]) && ((tmp >= dircos[1][1]) && (tmp >= dircos[2][2]
      )))  ) {
        tmp1 = (dircos[2][1]-dircos[1][2]);
        tmp2 = (dircos[0][2]-dircos[2][0]);
        tmp3 = (dircos[1][0]-dircos[0][1]);
        tmp4 = (1.+tmp);
    } else {
        if (((dircos[0][0] >= dircos[1][1]) && (dircos[0][0] >= dircos[2][2]))
          ) {
            tmp1 = (1.-(tmp-(2.*dircos[0][0])));
            tmp2 = (dircos[0][1]+dircos[1][0]);
            tmp3 = (dircos[0][2]+dircos[2][0]);
            tmp4 = (dircos[2][1]-dircos[1][2]);
        } else {
            if ((dircos[1][1] >= dircos[2][2])  ) {
                tmp1 = (dircos[0][1]+dircos[1][0]);
                tmp2 = (1.-(tmp-(2.*dircos[1][1])));
                tmp3 = (dircos[1][2]+dircos[2][1]);
                tmp4 = (dircos[0][2]-dircos[2][0]);
            } else {
                tmp1 = (dircos[0][2]+dircos[2][0]);
                tmp2 = (dircos[1][2]+dircos[2][1]);
                tmp3 = (1.-(tmp-(2.*dircos[2][2])));
                tmp4 = (dircos[1][0]-dircos[0][1]);
            }
        }
    }
    tmp = (1./sqrt(((tmp1*tmp1)+((tmp2*tmp2)+((tmp3*tmp3)+(tmp4*tmp4))))));
    *e1 = (tmp*tmp1);
    *e2 = (tmp*tmp2);
    *e3 = (tmp*tmp3);
    *e4 = (tmp*tmp4);
}

void ang2dc(double a1,
    double a2,
    double a3,
    double dircos[3][3])
{
    double cos1,cos2,cos3,sin1,sin2,sin3;

    cos1 = cos(a1);
    cos2 = cos(a2);
    cos3 = cos(a3);
    sin1 = sin(a1);
    sin2 = sin(a2);
    sin3 = sin(a3);
    dircos[0][0] = (cos2*cos3);
    dircos[0][1] = -(cos2*sin3);
    dircos[0][2] = sin2;
    dircos[1][0] = ((cos1*sin3)+(sin1*(cos3*sin2)));
    dircos[1][1] = ((cos1*cos3)-(sin1*(sin2*sin3)));
    dircos[1][2] = -(cos2*sin1);
    dircos[2][0] = ((sin1*sin3)-(cos1*(cos3*sin2)));
    dircos[2][1] = ((cos1*(sin2*sin3))+(cos3*sin1));
    dircos[2][2] = (cos1*cos2);
}

void quat2dc(double ie1,
    double ie2,
    double ie3,
    double ie4,
    double dircos[3][3])
{
    double e1,e2,e3,e4,e11,e22,e33,e44,norm;

    e11 = ie1*ie1;
    e22 = ie2*ie2;
    e33 = ie3*ie3;
    e44 = ie4*ie4;
    norm = sqrt(e11+e22+e33+e44);
    if (norm == 0.) {
        e4 = 1.;
        norm = 1.;
    } else {
        e4 = ie4;
    }
    norm = 1./norm;
    e1 = ie1*norm;
    e2 = ie2*norm;
    e3 = ie3*norm;
    e4 = e4*norm;
    e11 = e1*e1;
    e22 = e2*e2;
    e33 = e3*e3;
    dircos[0][0] = 1.-(2.*(e22+e33));
    dircos[0][1] = 2.*(e1*e2-e3*e4);
    dircos[0][2] = 2.*(e1*e3+e2*e4);
    dircos[1][0] = 2.*(e1*e2+e3*e4);
    dircos[1][1] = 1.-(2.*(e11+e33));
    dircos[1][2] = 2.*(e2*e3-e1*e4);
    dircos[2][0] = 2.*(e1*e3-e2*e4);
    dircos[2][1] = 2.*(e2*e3+e1*e4);
    dircos[2][2] = 1.-(2.*(e11+e22));
}

MatrixXd quat2dc(Vector4d st)
{
	double dc_cp[3][3];
	quat2dc(st[0],st[1],st[2],st[3],dc_cp);
	return matrix3_convert(dc_cp);
}

Vector3d dc2ang(MatrixXd dc)
{
	Vector3d stang;
	double dc_cp[3][3];
	matrix3_convert(dc,dc_cp);
	dc2ang(dc_cp,&stang[0],&stang[1],&stang[2]);
	return stang;
}

Vector3d quat2ang(Vector4d st)
{
	MatrixXd DC = quat2dc(st);
	return dc2ang(DC);
}

Vector4d ang2quat(Vector3d stang)
{
	MatrixXd DC = ang2dc(stang);
	return dc2quat(DC);
}

MatrixXd ang2dc(Vector3d stang)
{
	double dc_cp[3][3];
	ang2dc(stang[0],stang[1],stang[2],dc_cp);
	return matrix3_convert(dc_cp);
}

Vector4d dc2quat(MatrixXd dc)
{
	Vector4d st;
	double dc_cp[3][3];
	matrix3_convert(dc,dc_cp);
	dc2quat(dc_cp,&st[0],&st[1],&st[2],&st[3]);
	return st;
}

void matrix3_convert(MatrixXd& in, double out[3][3])
{
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            out[i][j]=in(i,j);
}

MatrixXd matrix3_convert(double in[3][3])
{
    MatrixXd out(3,3);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            out(i,j)=in[i][j];
    return out;
}

MatrixXd skew_symmetric(Vector3d in)
{
	MatrixXd res(3,3);
	res.setZero();
	res(0,1) = -in[2];
	res(1,0) = in[2];
	res(0,2) = in[1];
	res(2,0) = -in[1];
	res(1,2) = -in[0];
	res(2,1) = in[0];
	return res;
}

Vector3d skew_symmetric_inv(MatrixXd &in)
{
	Vector3d res;
	res[0] = in(2,1);
	res[1] = in(0,2);
	res[2] = in(1,0);
	return res;
}

MatrixXd make_cross_lh(Vector4d a)
{
	// would be P(a) * q = a x q
	MatrixXd P(4,4);
	P(0,0) =  a[3];
	P(0,1) = -a[2];
	P(0,2) =  a[1];
	P(0,3) =  a[0];
	P(1,0) =  a[2];
	P(1,1) =  a[3];
	P(1,2) = -a[0];
	P(1,3) =  a[1];
	P(2,0) = -a[1];
	P(2,1) =  a[0];
	P(2,2) =  a[3];
	P(2,3) =  a[2];
	P(3,0) = -a[0];
	P(3,1) = -a[1];
	P(3,2) = -a[2];
	P(3,3) =  a[3];
	return P;
}

Vector4d quat_cross(Vector4d a, Vector4d b)
{
	return make_cross_lh(a)*b;
}

Vector4d quat_cross(Vector4d a, Vector4d b, Vector4d c)
{
	return make_cross_lh(make_cross_lh(a)*b)*c;
}

Vector4d quat_normalize(Vector4d in)
{
	return in / quat_norm(in);
}

Vector4d quat_inverse(Vector4d in)
{
	VectorXd out = in;
	for(int i=0;i<3;i++) out[i] *= -1;
	return out;
}

double quat_dot(Vector4d a, Vector4d b)
{
	double sum = a[3]*b[3];
	for(int i=0;i<3;i++) sum -= a[i]*b[i];
	return sum;
}

Vector4d quat_mul(Vector4d a, Vector4d b)
{
	VectorXd res(4);
	res[0] =  a[1]*b[2] - a[2]*b[1] + a[0]*b[3] + a[3]*b[0];
	res[1] =  a[2]*b[0] - a[0]*b[2] + a[1]*b[3] + a[3]*b[1];
	res[2] =  a[0]*b[1] - a[1]*b[0] + a[2]*b[3] + a[3]*b[2];
	res[3] = -a[0]*b[0] - a[1]*b[1] - a[2]*b[2] + a[3]*b[3];
	if (res[3]<0)
		res *= -1;
	return res;
}

Vector4d quat_add(Vector4d a, Vector4d b)
{
	return quat_mul(a,b);
}

Vector4d quat_sub(Vector4d a, Vector4d b)
{
	return quat_mul(a, quat_inverse(b));
}

double quat_cos(Vector4d a, Vector4d b)
{
	return quat_dot(a,b)/quat_norm(a)/quat_norm(b);
}

double quat_norm(Vector4d in)
{
	return sqrt(quat_dot(in,quat_inverse(in)));
}

double quat_get_theta(Vector4d in)
{
	in /= quat_norm(in);
	double c = in[3];
	in[3] = 0;
	double s = quat_norm(in);
	return atan2(s,c)*2;
}

Vector4d quat_scale(Vector4d in, double d)
{
	in /= quat_norm(in);
	double c = in[3];
	in[3] = 0;
	double s = quat_norm(in);
	double theta = atan2(s,c)*2;
	if (s < 0.001)
		in.segment(0,3) = Vector3d(1.0,0.0,0.0);
	in.segment(0,3) *= sin(theta*d/2) * (s<0.001? 1.0 : 1.0/s);
	in[3] = cos(theta*d/2);
	return in;
}

double fact1(double x)	   //factorial function
{						   //Simply calculates factorial for denominator
	if(x==0 || x==1)
		return 1;
	else
		return x * fact1(x - 1);
}

double SinXoverX1(double x)	  //mySin function
{
	double sum = 0.0;
	for(int i = 0; i < 10; i++)
	{
		double top = pow(-1, i) * pow(x, 2 * i + 1 - 1);  //calculation for nominator
		double bottom = fact1(2 * i + 1);			  //calculation for denominator
		sum += top / bottom;
	}
	return sum;
}

Vector4d quat_exp(Vector4d in)
{
	Vector4d res;
	double amp = exp(in[3]);
	in[3] = 0;
	double theta = quat_norm(in);
	res[0] = amp * SinXoverX1(theta) * in[0];
	res[1] = amp * SinXoverX1(theta) * in[1];
	res[2] = amp * SinXoverX1(theta) * in[2];
	res[3] = amp * cos(theta);
	return res;
}

Vector4d quat_log(Vector4d in)
{
	Vector4d res;
	double amp = quat_norm(in);
	in = in / amp;
	double c = in[3];
	in[3] = 0;
	double s = quat_norm(in);
	if(s<1e-6)
	{
		res[0] = 0;
		res[1] = 0;
		res[2] = 0;
		res[3] = log(amp);
		return res;
	}
	double theta = 0;
	if(c>=0)
		theta = asin(s);
	else if(c<0 && s>=0)
		theta = acos(c);
	else
		theta = -acos(c);
	res[0] = theta * in[0]/s;
	res[1] = theta * in[1]/s;
	res[2] = theta * in[2]/s;
	res[3] = log(amp);
	return res;
}

Vector4d quat_pow(Vector4d in, double t)
{
	Vector4d I;
	I[0]=0;
	I[1]=0;
	I[2]=0;
	I[3]=1;
	if(t==0)
		return I;
	Vector4d l = quat_log(in);
	return quat_exp(l * t);
}

Vector4d quat_slerp(Vector4d q0, Vector4d q1, double t)
{
	return quat_cross(quat_pow(quat_cross(q1, quat_inverse(q0)),t),q0);
}

Vector4d quat_deriv(Vector3d in)
{
	Vector4d res;
	res[0] = in[0];
	res[1] = in[1];
	res[2] = in[2];
	res[3] = 0;
	return res;
}

Vector3d quat_diff(Vector4d a, Vector4d b)
{
	return quat_log(quat_sub(a, b)).segment(0,3);
}
