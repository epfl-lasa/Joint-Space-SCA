#include "Model.h"
using namespace std;

void Model::init()
{
    icub_air_init();

	// icub joint limits, obtained from urdf automatically
    qmin = Cvector(AIR_N_U-6);
    qmax = Cvector(AIR_N_U-6);
	taumin = Cvector(AIR_N_U-6);
    taumax = Cvector(AIR_N_U-6);

/* //urdf file
    qmin << -22, -39, -59, -40, -70, -55,
			-44, -119, -79, -125, -42, -24,
			-44, -119, -79, -125, -42, -24,
			-95,   0, -37,   5, -50, -65, -25,
			-95,   0, -37,   5, -50, -65, -25;
	qmax << 84,  39,  59,  30,  60,  55,
			132,  17,  79,  23*0-5,  21,  24,
			132,  17,  79,  23*0-5,  21,  24,
			5, 161, 100, 106,  50,  10,  25,
			5, 161, 100, 106,  50,  10,  25;
	taumax << 36, 80, 80, 20, 20, 20,
			  84, 84, 40, 30, 24, 11,
			  84, 84, 40, 30, 24, 11,
			  84, 84, 34, 20, 0.45, 0.65, 0.65,
			  84, 84, 34, 20, 0.45, 0.65, 0.65;
*/

	qmin <<  -22, -39, -59, -30, -20, -44,
			 -44, -17, -79, -125, -42, -24,
			 -44, -17, -79, -125, -42, -24,
			 -95,   0, -37,   5, -50, -65, -25,
			 -95,   0, -37,   5, -50, -65, -25;
	qmax <<   84,  39,  59,  22,  20,  44,
			  132, 119,  79,  23*0-5,  21,  24,
			  132, 119,  79,  23*0-5,  21,  24,
			  5, 161, 100, 106,  50,  10,  25,
			  5, 161, 100, 106,  50,  10,  25;
	taumax << 36, 80, 80, 20, 20, 20,
			  84, 84, 40, 30, 24, 11,
			  84, 84, 40, 30, 24, 11,
			  84, 84, 34, 20, 0.45, 0.65, 0.65,
			  84, 84, 34, 20, 0.45, 0.65, 0.65;

	// icub torque limits, obtained from urdf/sdf manually (WARNING)

	taumin = -taumax;
	ddqmin = Cvector::Ones(AIR_N_U-6) * (-500.0/180.0*M_PI);
    ddqmax = Cvector::Ones(AIR_N_U-6) *   500.0/180.0*M_PI;
}

void Model::set_state(double tt, Cvector& in_state, Cvector& in_stated)
{
    time = tt;
    //cout << in_state.transpose() <<endl;
    icub_air_state( tt, &in_state[0], &in_stated[0] );
}

Cvector Model::get_equivht()
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    icub_air_equivht( &tau[0] );
    return tau;
}

Cvector Model::get_comptrq(Cvector& udot)
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    icub_air_comptrq( &udot[0],  &tau[0] );
    return tau;
}

Cvector Model::get_fulltrq(Cvector& udot, Cvector& mult)
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    icub_air_fulltrq( &udot[0], &mult[0], &tau[0] );
    return tau;
}

Cmatrix Model::get_massmat()
{
    double mmat[AIR_N_U][AIR_N_U];
    icub_air_massmat( mmat );
    Cmatrix M = Cmatrix::Zero(AIR_N_U, AIR_N_U);
    for(int i=0;i<AIR_N_U;i++)
        for(int j=0;j<AIR_N_U;j++)
            M(i,j) = mmat[i][j];
    return M;
}

Cvector Model::get_frcmat()
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    icub_air_frcmat( &tau[0] );
    return tau;
}

Cvector Model::get_multtrq(Cvector & mult)
{
    Cvector tau = Cvector::Zero(AIR_N_U);
    icub_air_multtrq( &mult[0], &tau[0] );
    return tau;
}

Cvector3 Model::get_pos(int body, Cvector3 pt)
{
    Cvector3 loc_cp = zero_v3;
    icub_air_pos(body, &pt[0], &loc_cp[0]);
    return loc_cp;
}

Cvector3 Model::get_vel(int body, Cvector3 pt)
{
    // automatically gives the velocity in the global ref frame
    Cvector3 loc_cp = zero_v3;
    icub_air_vel(body, &pt[0], &loc_cp[0]);
    return loc_cp;
}

Cvector3 Model::get_acc(int body, Cvector3 pt)
{
    Cvector3 loc_cp = zero_v3;
    icub_air_acc(body, &pt[0], &loc_cp[0]);
    return loc_cp;
}

Cmatrix Model::get_orient(int body)
{
    double dircos[3][3];
    icub_air_orient(body, dircos);
    return matrix3_convert(dircos);
}

Cvector Model::get_orient_quat(int body)
{
    Cmatrix Orient = get_orient(body);
    return dc2quat(Orient);
}

Cvector3 Model::get_orient_ang(int body)
{
    Cmatrix Orient = get_orient(body);
    return dc2ang(Orient);
}

Cvector3 Model::get_angvel(int body)
{
    // gives the velocity in the local ref frame, we should multiply by body
    Cvector3 loc_cp;
    icub_air_angvel(body, &loc_cp[0]);
	icub_air_trans(body, &loc_cp[0], -1, &loc_cp[0]);
    return loc_cp;
}

Cvector3 Model::get_angacc(int body)
{
    // gives the velocity in the local ref frame, we should multiply by body
    Cvector3 loc_cp;
    icub_air_angacc(body, &loc_cp[0]);
	icub_air_trans(body, &loc_cp[0], -1, &loc_cp[0]);
    return loc_cp;
}

Cvector3 Model::get_trans(int frbod, Cvector3& ivec, int tobod)
{
    double loc_cp[3];
    icub_air_trans(frbod, &ivec[0], tobod, loc_cp);
    return Cvector3(loc_cp[0], loc_cp[1], loc_cp[2]);
}

Cmatrix Model::get_jacob(int body, Cvector3 pt, constraint_type type)
{
    // gives the velocity in the local ref frame, we should multiply
    double lin[3];
    double rot[3];

    Cmatrix J((type==CT_TRANSLATION || type==CT_ROTATION) ? 3 : 6, AIR_N_U);
    for(int i=0;i<AIR_N_U;i++)
    {
        icub_air_rel2cart(i, body, &pt[0], lin, rot);
        if(type==CT_TRANSLATION || type==CT_FULL)
        {
            icub_air_trans(body, lin, -1, lin);
            J(0, i) = lin[0];
            J(1, i) = lin[1];
            J(2, i) = lin[2];
        }
        if(type==CT_ROTATION || type==CT_FULL)
        {
            int shift = type==CT_FULL ? 3 : 0;
            icub_air_trans(body, rot, -1, rot);
            J(shift+0, i) = rot[0];
            J(shift+1, i) = rot[1];
            J(shift+2, i) = rot[2];
        }
    }
    return J;
}

void Model::set_acc(Cvector &acc)
{
    icub_air_setudot(&acc[0]);
}

void  Model::get_mom(Cvector3 &LM, Cvector3 &AM, double * KE)
{
    double ke;
    icub_air_mom(&LM[0], &AM[0], &ke);
    if(KE!=NULL)
        *KE = ke;
}

void Model::get_sys(Cvector3 &CM, Cmatrix &ICM, double * MTOT)
{
    double icm[3][3];
    double mtot;
    icub_air_sys(&mtot, &CM[0], icm);
    ICM = matrix3_convert(icm);
    *MTOT = mtot;
}

void Model::get_reac(Cmatrix &F, Cmatrix &T)
{
    double f[AIR_N_BODY][3];
    double t[AIR_N_BODY][3];
    icub_air_reac(f, t);
    F.resize(AIR_N_BODY,3);
    T.resize(AIR_N_BODY,3);
    for(int i=0;i<AIR_N_BODY;i++)
        for(int j=0;j<3;j++)
        {
            F(i,j)= f[i][j];
            T(i,j)= t[i][j];
        }
}

double Model::get_mass()
{
    double mass=0;
    double dm;
    for(int i=0;i<AIR_N_BODY;i++)
    {
        icub_air_getmass(i, &dm);
        mass += dm;
    }
    return mass;
}

double Model::get_mass(int body)
{
    double dm;
    icub_air_getmass(body, &dm);
    return dm;
}

Cmatrix Model::get_inertia(int body)
{
    double icm[3][3];
    icub_air_getiner(body, icm);
    return matrix3_convert(icm);
}

Cvector3 Model::get_cm()
{
    double mass=0;
    Cvector3 cm = zero_v3;
    double pt_cp[3] = {0,0,0};
    double loc_cp[3];
    for(int i=0;i<AIR_N_BODY;i++)
    {
        icub_air_getmass(i, &mass);
        icub_air_pos(i, pt_cp, loc_cp);
        cm[0] +=  loc_cp[0] * mass;
        cm[1] +=  loc_cp[1] * mass;
        cm[2] +=  loc_cp[2] * mass;
    }
    return cm/get_mass();
}

Cvector3 Model::get_cm_v()
{
    double mass=0;
    Cvector3 cm = zero_v3;
    double pt_cp[3] = {0,0,0};
    double loc_cp[3];
    for(int i=0;i<AIR_N_BODY;i++)
    {
        icub_air_getmass(i, &mass);
        icub_air_vel(i, pt_cp, loc_cp);
        cm[0] +=  loc_cp[0] * mass;
        cm[1] +=  loc_cp[1] * mass;
        cm[2] +=  loc_cp[2] * mass;
    }
    return cm/get_mass();
}

Cvector3 Model::get_cm_a()
{
    double mass=0;
    Cvector3 cm = zero_v3;
    double pt_cp[3] = {0,0,0};
    double loc_cp[3];
    for(int i=0;i<AIR_N_BODY;i++)
    {
        icub_air_getmass(i, &mass);
        icub_air_acc(i, pt_cp, loc_cp);
        cm[0] +=  loc_cp[0] * mass;
        cm[1] +=  loc_cp[1] * mass;
        cm[2] +=  loc_cp[2] * mass;
    }
    return cm/get_mass();
}

Cmatrix Model::get_cm_J()
{
    Cmatrix J = Cmatrix::Zero(3, AIR_N_U);
    double pt_cp[3]={0,0,0};
    double lin[3];
    double rot[3];

    double mass = 0;
    double M = get_mass();
    for(int j=0;j<AIR_N_BODY;j++)
    {
        icub_air_getmass(j, &mass);
        mass /= M;
        for(int i=0;i<AIR_N_U;i++)
        {
            icub_air_rel2cart(i, j, pt_cp, lin, rot);
			icub_air_trans(j, lin, -1, lin);
            J(0, i) += lin[0]*mass;
            J(1, i) += lin[1]*mass;
            J(2, i) += lin[2]*mass;
        }
    }
    return J;
}

void Model::check_consistency(Cvector joint_pos, Cvector joint_vel, Cvector joint_acc)
{
    // check consistency of joint and global variables, jacobians and sdfast functions
    ///////////////////////////// global vars
    // suppose the inertial fixed frame is called i
    // and the base frame is called b
    Cvector OR = ang2quat(Cvector3(0.5,0.8,0.7));
    Cmatrix rot = quat2dc(OR); // base orientation in i
    Cvector3 w(0.1, 0.5, -0.4); // base angular velocity expressed in b.
    Cvector3 v(0.01,0.02,0.03); // base linear velocity in i
    Cvector3 p(0.1,0.3,0.6); // base position in i
    Cvector3 a(-0.3,0.5,0.7); // base acceleration in i
    Cvector3 al(-0.6,0.4,0.9); // base angular acceleration in b.

    // now chose a link in the robot
    int body = 22;
    Cvector3 offset_r = zero_v3;

    ///////////////////////////// raw kinematics
    // setting base positions and orientations to zero in sdfast
    // in order to calculate end-effector's (chosen body) variables in base frame
    joint_pos[AIR_N_U] = 1;
    joint_pos.segment(0,6).setZero();
    joint_vel.segment(0,6).setZero();
    joint_acc.segment(0,6).setZero();

    for(int i=6;i<AIR_N_U;i++)
    {
        //joint_pos[i] += (double(rand()%1000)/1000.0*2.0-0.5);
        //joint_vel[i] += (double(rand()%1000)/1000.0*2.0-0.5);
        //joint_acc[i] += (double(rand()%1000)/1000.0*2.0-0.5);
    }

    set_state(0, joint_pos, joint_vel);
    set_acc(joint_acc);

    Cvector3 pos = get_pos(body, offset_r);
    Cvector3 vel = get_vel(body, offset_r);
    Cvector3 acc = get_acc(body, offset_r);
    Cmatrix ang = get_orient(body);
    Cvector3 angvel = get_angvel(body);
    Cvector3 angacc = get_angacc(body);

    ///////////////////////////// complex kinematics
    // setting base positions and orientations to desired values in sdfast
    // in order to calculate end-effector's (chosen body) variables in inertial frame
    joint_pos[AIR_N_U] = OR[3];
    joint_pos.segment(0,6) = vectorbig(p,OR.segment(0,3));
    joint_vel.segment(0,6) = vectorbig(v,w);
    joint_acc.segment(0,6) = vectorbig(a,al);
    set_state(0, joint_pos, joint_vel);
    set_acc(joint_acc);

    cout << endl << "/////////////check consistency//////////////" << endl;

    Cvector3 pos1 = p + rot * pos;
    cout << "pos1 " << pos1.transpose() << endl;
    Cvector3 pos2 = get_pos(body, offset_r);
    cout << "pos2 " << pos2.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 rot1 = dc2ang(rot * ang);
    cout << "rot1 " << rot1.transpose() << endl;
    Cvector3 rot2 = get_orient_ang(body);
    cout << "rot2 " << rot2.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 vel1 = v + rot * vel + rot * skew_symmetric(w) * pos;
    cout << "vel1 " << vel1.transpose() << endl;
    Cvector3 vel2 = get_vel(body, offset_r);
    cout << "vel2 " << vel2.transpose() << endl;
    Cvector3 vel3 = get_jacob(body,offset_r,CT_TRANSLATION) * joint_vel;
    cout << "vel3 " << vel3.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 angvel1 = rot * angvel + rot * w;
    cout << "angvel1 " << angvel1.transpose() << endl;
    Cvector3 angvel2 = get_angvel(body);
    cout << "angvel2 " << angvel2.transpose() << endl;
    Cvector3 angvel3 = get_jacob(body,offset_r,CT_ROTATION) * joint_vel;
    cout << "angvel3 " << angvel3.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 acc1 = a + rot * acc +
                    rot * skew_symmetric(w) * vel * 2 +
                    rot * skew_symmetric(w) * skew_symmetric(w) * pos +
                    rot * skew_symmetric(al) * pos;
    cout << "acc1 " << acc1.transpose() << endl;
    Cvector3 acc2 = get_acc(body, offset_r);
    cout << "acc2 " << acc2.transpose() << endl;

    Cvector copy = joint_acc * 0;
    set_acc(copy);
    Cvector3 acc3 = get_acc(body, offset_r);
    acc3 += get_jacob(body,offset_r,CT_TRANSLATION) * joint_acc;
    set_acc(joint_acc);
    cout << "acc3 " << acc3.transpose() << endl;

    cout << "////////////////////////////////////////////" << endl;

    Cvector3 angacc1 = rot * skew_symmetric(w) * angvel +
                      rot * angacc +
                      rot * al;
    cout << "angacc1 " << angacc1.transpose() << endl;
    Cvector3 angacc2 = get_angacc(body);
    cout << "angacc2 " << angacc2.transpose() << endl;

    set_acc(copy);
    Cvector3 angacc3 = get_angacc(body);
    angacc3 += get_jacob(body,offset_r,CT_ROTATION) * joint_acc;
    set_acc(joint_acc);
    cout << "angacc3 " << angacc3.transpose() << endl;

    cout << "/////////////check consistency//////////////" << endl << endl;

// conclusion:
// in the state vector, pos, vel and acc are in global frame
// in the state vector, OR is global, but w and al are local, in base frame
}

/*************************************************************************************/

Cmatrix quat2dc(Cvector st)
{
    double dc_cp[3][3];
    icub_air_quat2dc(st[0],st[1],st[2],st[3],dc_cp);
    return matrix3_convert(dc_cp);
}

Cvector3 dc2ang(Cmatrix dc)
{
    Cvector3 stang;
    double dc_cp[3][3];
    matrix3_convert(dc,dc_cp);
    icub_air_dc2ang(dc_cp,&stang[0],&stang[1],&stang[2]);
    return stang;
}

Cvector3 quat2ang(Cvector st)
{
    Cmatrix DC = quat2dc(st);
    return dc2ang(DC);
}

Cvector ang2quat(Cvector3 stang)
{
    Cmatrix DC = ang2dc(stang);
    return dc2quat(DC);
}

Cmatrix ang2dc(Cvector3 stang)
{
    double dc_cp[3][3];
    icub_air_ang2dc(stang[0],stang[1],stang[2],dc_cp);
    return matrix3_convert(dc_cp);
}

Cvector dc2quat(Cmatrix dc)
{
    Cvector st(4);
    double dc_cp[3][3];
    matrix3_convert(dc,dc_cp);
    icub_air_dc2quat(dc_cp,&st[0],&st[1],&st[2],&st[3]);
    return st;
}

Cmatrix skew_symmetric(Cvector3 in)
{
    Cmatrix res(3,3);
    res.setZero();
    res(0,1) = -in[2];
    res(1,0) = in[2];
    res(0,2) = in[1];
    res(2,0) = -in[1];
    res(1,2) = -in[0];
    res(2,1) = in[0];
    return res;
}

Cvector3 skew_symmetric_inv(Cmatrix &in)
{
    Cvector3 res;
    res[0] = in(2,1);
    res[1] = in(0,2);
    res[2] = in(1,0);
    return res;
}

/*************************************************************************************/

Cmatrix make_cross_lh(Cvector a)
{
    // would be P(a) * q = a x q
    Cmatrix P(4,4);
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

Cvector quat_cross(Cvector a, Cvector b)
{
    return make_cross_lh(a)*b;
}

Cvector quat_cross(Cvector a, Cvector b, Cvector c)
{
    return make_cross_lh(make_cross_lh(a)*b)*c;
}

Cvector quat_normalize(Cvector in)
{
    return in / quat_norm(in);
}

Cvector quat_inverse(Cvector in)
{
    Cvector out = in;
    for(int i=0;i<3;i++) out[i] *= -1;
    return out;
}

double quat_dot(Cvector a, Cvector b)
{
    double sum = a[3]*b[3];
    for(int i=0;i<3;i++) sum -= a[i]*b[i];
    return sum;
}

Cvector quat_mul(Cvector a, Cvector b)
{
    Cvector res(4);
    res[0] =  a[1]*b[2] - a[2]*b[1] + a[0]*b[3] + a[3]*b[0];
    res[1] =  a[2]*b[0] - a[0]*b[2] + a[1]*b[3] + a[3]*b[1];
    res[2] =  a[0]*b[1] - a[1]*b[0] + a[2]*b[3] + a[3]*b[2];
    res[3] = -a[0]*b[0] - a[1]*b[1] - a[2]*b[2] + a[3]*b[3];
	if (res[3]<0)
		res *= -1;
    return res;
}

Cvector quat_add(Cvector a, Cvector b)
{
    return quat_mul(a,b);
}

Cvector quat_sub(Cvector a, Cvector b)
{
    return quat_mul(a, quat_inverse(b));
}

double quat_cos(Cvector a, Cvector b)
{
    return quat_dot(a,b)/quat_norm(a)/quat_norm(b);
}

double quat_norm(Cvector in)
{
    return sqrt(quat_dot(in,quat_inverse(in)));
}

double quat_get_theta(Cvector in)
{
    in /= quat_norm(in);
    double c = in[3];
    in[3] = 0;
    double s = quat_norm(in);
    return atan2(s,c)*2;
}

double fact1(double x)       //factorial function
{                           //Simply calculates factorial for denominator
    if(x==0 || x==1)
        return 1;
    else
        return x * fact1(x - 1);
}

double SinXoverX1(double x)      //mySin function
{
    double sum = 0.0;
    for(int i = 0; i < 10; i++)
    {
        double top = pow(-1, i) * pow(x, 2 * i + 1 - 1);  //calculation for nominator
        double bottom = fact1(2 * i + 1);              //calculation for denominator
        sum += top / bottom;
    }
    return sum;
}

Cvector quat_exp(Cvector in)
{
    Cvector res(4);
    double amp = exp(in[3]);
    in[3] = 0;
    double theta = quat_norm(in);
    res[0] = amp * SinXoverX1(theta) * in[0];
    res[1] = amp * SinXoverX1(theta) * in[1];
    res[2] = amp * SinXoverX1(theta) * in[2];
    res[3] = amp * cos(theta);
    return res;
}

Cvector quat_log(Cvector in)
{
    Cvector res(4);
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

Cvector quat_pow(Cvector in, double t)
{
    Cvector I(4);
    I[0]=0;
    I[1]=0;
    I[2]=0;
    I[3]=1;
    if(t==0)
        return I;
    Cvector l = quat_log(in);
    return quat_exp(l * t);
}

Cvector quat_slerp(Cvector q0, Cvector q1, double t)
{
    return quat_cross(quat_pow(quat_cross(q1, quat_inverse(q0)),t),q0);
}

Cvector quat_deriv(Cvector3 in)
{
    Cvector res(4);
    res[0] = in[0];
    res[1] = in[1];
    res[2] = in[2];
    res[3] = 0;
    return res;
}



