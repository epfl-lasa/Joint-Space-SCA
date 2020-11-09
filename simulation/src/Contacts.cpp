#include "Contacts.h"
using namespace std;

char names[5][6+1] = {"pelvis", "l_foot", "r_foot", "l_hand", "r_hand"};

Contact::Contact()
{
    initialize_contact();
}

Contact::~Contact()
{
    body = 0;
}

void Contact::init(Contact_Name Name,
            int Body,
            Vector3d Offset,
            constraint_type Contact_type,
            Point_Status Status,
			double length,
			double width)
{
    initialize_contact();
    name = Name;
    body = Body;
    offset = Offset;
    contact_type = Contact_type;
    status = Status;
	w_x = Vector3d(-1,1,0) * length / 2.0;
	w_y = Vector3d(-1,1,0) * width / 2.0;
}

void Contact::initialize_contact()
{
    name = (Contact_Name)-1;
    body = 0;
    offset = zero_v3;
    contact_type = CT_FULL;
    status = PS_NO_CONTROL;
    used_for_odometry = false;

    Geom_Point * gp[3] = {&p, &init_p, &ref_p};
    for(int i=0;i<3;i++)
    {
        gp[i]->pos = zero_v7;
        gp[i]->vel = zero_v6;
    }

    Dyn_Point * dp[2] = {&T, &R};
    for(int i=0;i<2;i++)
    {
        dp[i]->J = MatrixXd(1,1).setZero();
        dp[i]->F_est = zero_v3;
        dp[i]->F_ref = zero_v3;
        dp[i]->F_sens = zero_v3;

        dp[i]->F_cost = zero_v3;
        dp[i]->K = Vector3d(20, 3, 1);
        dp[i]->slack_cost = zero_v3;
    }

    cop = zero_v3;
    n1 = Vector3d(1,0,0);
    n2 = Vector3d(0,1,0);
    n3 = Vector3d(0,0,1);
    w_x = Vector3d(-100,100,0);
    w_y = Vector3d(-100,100,0);
    mu = 1;
    muR = 1;
}

Contact_Manager::Contact_Manager()
{
	ifCoM = true;
    model.init();
    EE.resize(NUM_Contact);

	// this combination of contact points is fixed since the jacobian in the IK solver has hard-coded sparsity pattern
	EE[CN_CM].init(CN_CM, body_base,   offset_base,   CT_FULL, 		PS_FLOATING,  0              , 0             );
	EE[CN_LF].init(CN_LF, body_l_foot, offset_l_foot, CT_FULL, 		PS_CONTACTED, foot_length*0.8, foot_width*0.8);
	EE[CN_RF].init(CN_RF, body_r_foot, offset_r_foot, CT_FULL, 		PS_CONTACTED, foot_length*0.8, foot_width*0.8);
	EE[CN_LH].init(CN_LH, body_l_hand, offset_l_hand, CT_FULL, 		PS_FLOATING,  hand_length*0.8, hand_width*0.8);
	EE[CN_RH].init(CN_RH, body_r_hand, offset_r_hand, CT_FULL, 		PS_FLOATING,  hand_length*0.8, hand_width*0.8);
	EE[CN_TO].init(CN_TO, body_torso , offset_torso , CT_ROTATION, 	PS_FLOATING,  0              , 0             );
	EE[CN_HD].init(CN_HD, body_head  , offset_head  , CT_ROTATION, 	PS_FLOATING,  0              , 0             );

    task_costs();
}

Contact_Manager::~Contact_Manager()
{
    EE.clear();
}

void Contact_Manager::update_kinematics()
{
	// assumes model.set_state is called before
    for(unsigned int i=0;i<EE.size();i++)
    {
        EE[i].p.pos  = model.get_trans7(EE[i].body, EE[i].offset);
        EE[i].p.vel  = model.get_vel6(EE[i].body, EE[i].offset);
        EE[i].T.J    = model.get_jacob(EE[i].body, EE[i].offset, CT_TRANSLATION);
        EE[i].R.J    = model.get_jacob(EE[i].body, EE[i].offset, CT_ROTATION);
        if(i == 0 && ifCoM)
        {
            EE[i].p.pos.segment(0,3) = model.get_cm();
            EE[i].p.vel.segment(0,3) = model.get_cm_v();
            EE[i].T.J    = model.get_cm_J();
        }
        // a rough estimation of the contact normal.
        EE[i].n1 = model.get_pos(EE[i].body,Vector3d(1,0,0)) - model.get_pos(EE[i].body,zero_v3);
        EE[i].n2 = model.get_pos(EE[i].body,Vector3d(0,1,0)) - model.get_pos(EE[i].body,zero_v3);
        EE[i].n3 = model.get_pos(EE[i].body,Vector3d(0,0,1)) - model.get_pos(EE[i].body,zero_v3);
    }
}

void Contact_Manager::save_initial_contacts()
{
    for(unsigned int i=0;i<EE.size();i++)
    {
        EE[i].init_p.pos = model.get_trans7(EE[i].body, EE[i].offset);
        EE[i].init_p.vel = zero_v6;
        if(i == 0 && ifCoM)
            EE[i].init_p.pos.segment(0,3) = model.get_cm();
    }
}

unsigned int Contact_Manager::size()
{
    return EE.size();
}

VectorXd Contact_Manager::get_mult()
{
    VectorXd mult = VectorXd::Zero(AIR_N_U);
    for(unsigned int i=1;i<EE.size();i++)
    {
		if(EE[i].contact_type==CT_FULL || EE[i].contact_type==CT_TRANSLATION)
        	mult += EE[i].T.J.transpose() * EE[i].T.F_est;
		if(EE[i].contact_type==CT_FULL || EE[i].contact_type==CT_ROTATION)
        	mult += EE[i].R.J.transpose() * EE[i].R.F_est;
    }
    return mult;
}

VectorXd Contact_Manager::get_virtual_mult()
{
    VectorXd mult = VectorXd::Zero(AIR_N_U);
    for(unsigned int i=0;i<EE.size();i++)
    {
		if(EE[i].contact_type==CT_FULL || EE[i].contact_type==CT_TRANSLATION)
	        mult += EE[i].T.J.transpose() * EE[i].T.F_ref;
		if(EE[i].contact_type==CT_FULL || EE[i].contact_type==CT_ROTATION)
	        mult += EE[i].R.J.transpose() * EE[i].R.F_ref;
    }
    return mult;
}

void Contact_Manager::control(Contact_Name i, Geom_Point ref_pos)
{
    if(EE[i].contact_type==CT_FULL || EE[i].contact_type==CT_TRANSLATION)
        EE[i].T.F_ref    = EE[i].T.K[0] * (EE[i].ref_p.pos - EE[i].p.pos).segment(0,3)
                            + EE[i].T.K[1] * (EE[i].ref_p.vel - EE[i].p.vel).segment(0,3);
    if(EE[i].contact_type==CT_FULL || EE[i].contact_type==CT_ROTATION)
        EE[i].R.F_ref    = EE[i].R.K[0] * quat_log(quat_sub(EE[i].ref_p.pos.segment(3,4), EE[i].p.pos.segment(3,4))).segment(0,3)
                            + EE[i].R.K[1] * (EE[i].ref_p.vel - EE[i].p.vel).segment(3,3);

}

void Contact_Manager::load_tasks(VectorXd com, VectorXd lf, VectorXd rf, VectorXd lh, VectorXd rh)
{
    EE[CN_CM].ref_p.pos = com;
	EE[CN_CM].ref_p.pos = com;
	EE[CN_TO].ref_p.pos = com;
	EE[CN_HD].ref_p.pos = com;

	EE[CN_LF].ref_p.pos = lf;
	EE[CN_RF].ref_p.pos = rf;

	EE[CN_LH].ref_p.pos = lh;
	EE[CN_RH].ref_p.pos = rh;
}

void Contact_Manager::load_tasks(VectorXd com, VectorXd lf, VectorXd rf, VectorXd hands)
{
    load_tasks(com, lf, rf, hands.segment(0,7), hands.segment(7,7));
}

void Contact_Manager::task_costs()
{
	EE[CN_CM].T.slack_cost = Vector3d(-1,-1,0.01);
    EE[CN_CM].R.slack_cost = Vector3d(1,1,1) * 0.01;
    EE[CN_TO].R.slack_cost = Vector3d(1,1,1) * 0.01;
    EE[CN_HD].R.slack_cost = Vector3d(1,1,1) * 0.01;

    EE[CN_LF].T.slack_cost = Vector3d(-1,-1,-1);
    EE[CN_RF].T.slack_cost = Vector3d(-1,-1,-1);
    EE[CN_LF].R.slack_cost = Vector3d(-1,-1,-1);
    EE[CN_RF].R.slack_cost = Vector3d(-1,-1,-1);

    EE[CN_LH].T.slack_cost = Vector3d(1,1,1);
    EE[CN_RH].T.slack_cost = Vector3d(1,1,1);
    EE[CN_LH].R.slack_cost = 0.05*Vector3d(1,1,1);
    EE[CN_RH].R.slack_cost = 0.05*Vector3d(1,1,1);
}

void print_VectorXd(VectorXd x, double precision)
{
	for(int i=0; i<x.size();i++)
		cout << fixed << setprecision( precision ) << std::setw( 11 ) << x[i];
	cout << endl;
}

void Contact_Manager::print_forces()
{
    printf("----- Contact forces -------------------------------------- \n");
    for(unsigned int i=0;i<EE.size();i++)
	{
		cout << names[i];
		print_VectorXd(vectorbig(EE[i].T.F_sens, EE[i].R.F_sens), 2);
	}
}

void Contact_Manager::print_IK_errors()
{
    printf("----- Contact IK position errors -------------------------- \n");
    for(unsigned int i=0;i<EE.size();i++)
	{
		cout << names[i];
		print_VectorXd(vectorbig(EE[i].ref_p.pos.segment(0,3)- EE[i].p.pos.segment(0,3), 
                                 quat_sub(EE[i].ref_p.pos.segment(3,4), EE[i].p.pos.segment(3,4)).segment(0,3)), 3);
	}
}