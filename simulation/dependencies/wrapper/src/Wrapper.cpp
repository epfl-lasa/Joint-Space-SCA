#include "Wrapper.h"

using namespace Eigen;
using namespace std;

void *IMU_thread(void * input)
{
	while(1)
	{	
		Wrapper * d = ((Wrapper*)input);
		pthread_mutex_lock(&(d->mutex));
		d->imu.update(d->EULER, d->LINACC, d->ANGVEL);
		pthread_mutex_unlock(&(d->mutex));
		usleep(1*1000);
	}
}

Matrix3d Wrapper::rotmatrix(Vector3d a)
{
	// a: roll/pitch/yaw or XYZ Tait-Bryan angles
	// https://en.wikipedia.org/wiki/Euler_angles
	double cos1 = cos(a[0]);
	double cos2 = cos(a[1]);
	double cos3 = cos(a[2]);
	double sin1 = sin(a[0]);
	double sin2 = sin(a[1]);
	double sin3 = sin(a[2]);

	Matrix3d dircos;
	dircos(0,0) = (cos2*cos3);
	dircos(0,1) = -(cos2*sin3);
	dircos(0,2) = sin2;
	dircos(1,0) = ((cos1*sin3)+(sin1*(cos3*sin2)));
	dircos(1,1) = ((cos1*cos3)-(sin1*(sin2*sin3)));
	dircos(1,2) = -(cos2*sin1);
	dircos(2,0) = ((sin1*sin3)-(cos1*(cos3*sin2)));
	dircos(2,1) = ((cos1*(sin2*sin3))+(cos3*sin1));
	dircos(2,2) = (cos1*cos2);
	return dircos;
}

VectorXd Wrapper::dc2quat(Matrix3d dircos)
{
	double tmp,tmp1,tmp2,tmp3,tmp4,temp[10];

	tmp = (dircos(0,0)+(dircos(1,1)+dircos(2,2)));
	if (((tmp >= dircos(0,0)) && ((tmp >= dircos(1,1)) && (tmp >= dircos(2,2)))))
	{
		tmp1 = (dircos(2,1)-dircos(1,2));
		tmp2 = (dircos(0,2)-dircos(2,0));
		tmp3 = (dircos(1,0)-dircos(0,1));
		tmp4 = (1.+tmp);
	} 
	else 
	{
		if (((dircos(0,0) >= dircos(1,1)) && (dircos(0,0) >= dircos(2,2))))
		{
			tmp1 = (1.-(tmp-(2.*dircos(0,0))));
			tmp2 = (dircos(0,1)+dircos(1,0));
			tmp3 = (dircos(0,2)+dircos(2,0));
			tmp4 = (dircos(2,1)-dircos(1,2));
		}
		else
		{
			if((dircos(1,1) >= dircos(2,2)))
			{
				tmp1 = (dircos(0,1)+dircos(1,0));
				tmp2 = (1.-(tmp-(2.*dircos(1,1))));
				tmp3 = (dircos(1,2)+dircos(2,1));
				tmp4 = (dircos(0,2)-dircos(2,0));
			}
			else
			{
				tmp1 = (dircos(0,2)+dircos(2,0));
				tmp2 = (dircos(1,2)+dircos(2,1));
				tmp3 = (1.-(tmp-(2.*dircos(2,2))));
				tmp4 = (dircos(1,0)-dircos(0,1));
			}
		}
	}
	tmp = (1./sqrt(((tmp1*tmp1)+((tmp2*tmp2)+((tmp3*tmp3)+(tmp4*tmp4))))));

	VectorXd e = VectorXd::Zero(4);
	e[0] = (tmp*tmp1);
	e[1] = (tmp*tmp2);
	e[2] = (tmp*tmp3);
	e[3] = (tmp*tmp4);
	return e;
}

VectorXd Wrapper::quat_mul(VectorXd a, VectorXd b)
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

Wrapper::Wrapper()
{
}

Wrapper::~Wrapper()
{
}

int Wrapper::checkRobot(int argc, char *argv[])
{
	params.fromCommand(argc, argv);
	if (!params.check("robot"))
	{
		fprintf(stderr, "Please specify the name of the robot\n");
		fprintf(stderr, "--robot name (e.g. icubSim)\n");
		return 1;
	}
	robotName = params.find("robot").asString();
	robotName = "/" + robotName;
	return 0;
}

int Wrapper::setupSensorConnector(sensorConnector &F, string moduleName)
{
	F.port.open("/receiver" + robotName + moduleName);
	yarp::os::Network::connect(moduleName.c_str(), F.port.getName().c_str());
	return 0;
}

int Wrapper::setupJointConnector(jointConnector &C, string moduleName, int size)
{
	string remotePorts = moduleName;
	string localPorts = "/receiver" + robotName + moduleName;

	yarp::os::Property options;
	options.put("device", "remote_controlboard");
	options.put("local", localPorts);
	options.put("remote", remotePorts);

	C.robotDevice.open(options);
	if (!C.robotDevice.isValid()) 
	{
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", yarp::dev::Drivers::factory().toString().c_str());
		return 1;
	}

	bool ok;
	ok = C.robotDevice.view(C.posdirect);
	ok = ok && C.robotDevice.view(C.pos);
	ok = ok && C.robotDevice.view(C.encs);
	ok = ok && C.robotDevice.view(C.ictrl);
	ok = ok && C.robotDevice.view(C.iint);
	ok = ok && C.robotDevice.view(C.iimp);
	ok = ok && C.robotDevice.view(C.itrq);
	ok = ok && C.robotDevice.view(C.ipid);
	ok = ok && C.robotDevice.view(C.ilimit);
	if (!ok) 
	{
		printf("Problems acquiring interfaces\n");
		return 1;
	}

	C.number = size;
	return 0;
}

void Wrapper::initialize()
{
	/* Index reference and a natural posture of icub           */
	/* global positions                                        */
	/* pelvis pos:     pos[0] pos[1] pos[2]                    */
	/* pelvis rot:     pos[3] pos[4] pos[5] pos[38]            */
	/*                 // right      // left                   */
	/* head                   pos[11]                          */
	/* neck roll              pos[10]                          */
	/* neck pitch             pos[9]                           */
	/* shoulder pitch  pos[31]       pos[24]                   */
	/* shoulder roll   pos[32]       pos[25]                   */
	/* shoulder yaw    pos[33]       pos[26]                   */
	/* elbow           pos[34]       pos[27]                   */
	/* forearm yaw     pos[35]       pos[28]                   */
	/* forearm roll    pos[36]       pos[29]                   */
	/* forearm pitch   pos[37]       pos[30]                   */
	/* torso pitch            pos[6]                           */
	/* torso roll             pos[7]                           */
	/* torso yaw              pos[8]                           */
	/* hip pitch       pos[18]      pos[12]                    */
	/* hip roll        pos[19]       pos[13]                   */
	/* hip yaw         pos[20]       pos[14]                   */
	/* knee            pos[21]       pos[15]                   */
	/* ankle pitch     pos[22]       pos[16]                   */
	/* ankle roll      pos[23]       pos[17]                   */

	map = MatrixXd(6,7);
	map <<  2 ,1 ,0 ,-1,-1,-1,-1,
			3 ,4 ,5 ,-1,-1,-1,-1,
			6 ,7 ,8 ,9 ,10,11,-1,
			12,13,14,15,16,17,-1,
			18,19,20,21,22,23,24,
			25,26,27,28,29,30,31;

	// variable initialization
	startTime = 0;
	time      = 0;
	dt        = minTimeStep;
	linacc    = Vector3d::Zero(3);
	angvel    = Vector3d::Zero(3);
	R         = MatrixXd::Identity(3, 3);
	for(int i=0; i<4; i++)
		for(int j=0; j<3; j++)
		{
			FTsensor[i][0][j] = 0;
			FTsensor[i][1][j] = 0;
		}
	for(int k=0;k<10;k++)
		Object[k] = VectorXd::Zero(7);

	//external wrench
	string portname = robotName + "/applyExternalWrench/rpc:i";
	ExternalWrenchPort.port.open(robotName + "/applyExternalWrench:o");
	yarp::os::Network::connect(ExternalWrenchPort.port.getName().c_str(), portname.c_str());	

	int status = 0;

	// controllers
	status |= setupJointConnector(JointPort[0], robotName + "/torso", 3);
	status |= setupJointConnector(JointPort[1], robotName + "/head", 3);
	status |= setupJointConnector(JointPort[2], robotName + "/left_leg", 6);
	status |= setupJointConnector(JointPort[3], robotName + "/right_leg", 6);
	status |= setupJointConnector(JointPort[4], robotName + "/left_arm", 7);
	status |= setupJointConnector(JointPort[5], robotName + "/right_arm", 7);

	// other sensors
	#ifdef ICUB5
		// port definition in old icub
		status |= setupSensorConnector(SensorPort[0], robotName + "/left_leg/analog:o");
		status |= setupSensorConnector(SensorPort[1], robotName + "/right_leg/analog:o");
	#elif defined(ICUBSIM) || defined(ICUB33)
		status |= setupSensorConnector(SensorPort[0], robotName + "/left_foot/analog:o");
		status |= setupSensorConnector(SensorPort[1], robotName + "/right_foot/analog:o");
	#endif
	status |= setupSensorConnector(SensorPort[2], robotName + "/left_arm/analog:o");
	status |= setupSensorConnector(SensorPort[3], robotName + "/right_arm/analog:o");

	#if defined(ICUB5) || defined(ICUB33)
		// initialize IMU
		printf("Check which tty is associated to the IMU:\n");
		printf("ls /dev/ttyACM (now press tab)\n");
		printf("sudo chmod 666 /dev/ttyACM0\n");
		printf("sudo adduser #user dialout\n");
		u32 com_port, baudrate;
		com_port = 0;
		baudrate = 115200;
		imu.init(com_port, baudrate);
		pthread_t thread1;
		pthread_mutex_init (&mutex , NULL);
		int i1 = pthread_create( &thread1, NULL, IMU_thread, (void*)(this));
	#elif defined(ICUBSIM)
		status |= setupSensorConnector(SensorPort[4], robotName + "/inertial");
		status |= setupSensorConnector(SensorPort[5], "/clock");
	#endif

	if(status != 0)
		cout << "WARNING: couldn't set up some yarp ports" << endl;
}

void Wrapper::initObject(int k, string name)
{
	if(k<0 || k>=10)
	{
		cout << "WARNING: not supported object number" << endl;
		return;
	}
	if(!ObjectPorts[k].port.isClosed())
		ObjectPorts[k].port.close();
	setupSensorConnector(ObjectPorts[k], name);
	ObjectFilter[k].init(7,10);
}

VectorXd Wrapper::readObject(int k)
{
	// this function only updates the object k
	// if there was no update, the previou value is returned
	if(k<0 || k>=10 || ObjectPorts[k].port.isClosed())
		cout << "WARNING: not supported object number or port closed" << endl;
	else
	{
		ObjectPorts[k].values = ObjectPorts[k].port.read(false);
		if(ObjectPorts[k].values)
			for(int i=0; i<7; i++)
				Object[k][i] = ObjectPorts[k].values->get(i).asDouble();
	}
	ObjectFilter[k].update(Object[k]);
	// can call ObjectFilter[k].if_frozen_signal() to check if data is coming
	return Object[k];
}

void Wrapper::close()
{
	for(int i=0; i<6; i++)
        JointPort[i].robotDevice.close();
	for(int i=0; i<6; i++)
        SensorPort[i].port.close();
	#if defined(ICUB5) || defined(ICUB33)
		pthread_mutex_destroy(&mutex);
	#endif
}

void Wrapper::initializeJoint(VectorXd mode, VectorXd freeze)
{
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			if(mode[map(i,j)] == 0 && freeze[map(i,j)] == 0)
			{
				JointPort[i].ictrl->setControlMode(j,VOCAB_CM_POSITION);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_STIFF);
			}
			else if(mode[map(i,j)] == 1 && freeze[map(i,j)] == 0)
			{
				JointPort[i].ictrl->setControlMode(j,VOCAB_CM_POSITION_DIRECT);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_STIFF);
			}
			else if(mode[map(i,j)] == 2 && freeze[map(i,j)] == 0)
			{
				JointPort[i].ictrl->setControlMode(j, VOCAB_CM_TORQUE);
				JointPort[i].iint->setInteractionMode(j, yarp::dev::VOCAB_IM_COMPLIANT);
			}
		}
	}
}

void Wrapper::controlJoint(VectorXd mode, VectorXd freeze, VectorXd ref_pos, VectorXd ref_tau)
{
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			if(mode[map(i,j)] == 0 && freeze[map(i,j)] == 0)
			{	
				JointPort[i].pos->positionMove(j, ref_pos[map(i,j)] / M_PI * 180);
			}
			else if(mode[map(i,j)] == 1 && freeze[map(i,j)] == 0)
			{
				JointPort[i].posdirect->setPosition(j, ref_pos[map(i,j)] / M_PI * 180);
			}
			else if(mode[map(i,j)] == 2 && freeze[map(i,j)] == 0)
			{
				JointPort[i].itrq->setRefTorque(j, ref_tau[map(i,j)]);
			}
		}
	}
}

void Wrapper::setPidJoint(int k, double kp, double kd, double ki)
{
	MatrixXd gain = MatrixXd(32,3);
	#ifdef ICUB5
		gain << 32000.00, 6000.00, 60.00,
				32000.00, 6000.00, 60.00,
				32000.00, 6000.00, 60.00,
				50.00, 500.00, 1.00,
				50.00, 500.00, 1.00,
				100.00, 700.00, 2.00,
				32000.00, 100.00, 60.00,
				-32000.00, -100.00, -60.00,
				32000.00, 100.00, 60.00, 
				-32000.00, -100.00, -60.00,
				-32000.00, -100.00, -60.00, 
				-32000.00, -100.00, -60.00, 
				32000.00, 100.00, 60.00, 
				-32000.00, -100.00, -60.00,
				32000.00, 100.00, 60.00, 
				-32000.00, -100.00, -60.00,
				-32000.00, -100.00, -60.00, 
				-32000.00, -100.00, -60.00, 
				32000.00, 50.00, 60.00, 
				32000.00, 50.00, 60.00, 
				10000.00, 0.00, 10.00, 
				32000.00, 20.00, 60.00,
				200.00, 1000.00, 1.00, 
				100.00, 100.00, 2.00, 
				100.00, 100.00, 2.00, 
				32000.00, 50.00, 60.00,
				32000.00, 50.00, 60.00, 
				10000.00, 0.00, 10.00, 
				32000.00, 20.00, 60.00, 
				200.00, 1000.00, 1.00,
				100.00, 100.00, 2.00,
				100.00, 100.00, 2.00;
	#elif defined(ICUB33)
		gain << -1066.66, 0.00, -14222.18, 
				-1066.66, 0.00, -10666.64, 
				-711.11, 0.00, -7111.09, 
				-300.00, -10.00, -100.00, 
				300.00, 10.00, 100.00, 
				1100.00, 0.00, 0.00, 
				-1066.66, 0.00, -10666.64, 
				2066.66, 0.00, 14222.18, 
				-711.11, 0.00, -7111.09, 
				-1066.66, 0.00, -1066.64, 
				2200.00, 0.00, 0.09, 
				2200.00, 0.00, 0.09, 
				1066.66, 0.00, 10666.64, 
				-2066.66, 0.00, -14222.18, 
				711.11, 0.00, 7111.09, 
				1066.66, 0.00, 1066.64, 
				-2105.00, 0.00, -0.09, 
				-2310.00, 0.00, -0.09, 
				711.11, 0.00, 7111.09, 
				1066.66, 0.00, 10666.64, 
				711.11, 0.00, 7111.09, 
				1066.66, 0.00, 10666.64, 
				-200.00, 0.00, -200.00, 
				-500.00, 0.00, -50.00, 
				-500.00, 0.00, -50.00, 
				-711.11, 0.00, -7111.09, 
				-1066.66, 0.00, -10666.64, 
				-711.11, 0.00, -7111.09, 
				-1066.66, 0.00, -10666.64, 
				200.00, 0.00, 200.00, 
				500.00, 0.00, 50.00, 
				500.00, 0.00, 50.00;
	#elif defined(ICUBSIM)
		gain << 1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.122, 0.003, 
				1.745, 0.122, 0.003, 
				1.745, 0.122, 0.003, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				17.453, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174, 
				1.745, 0.174, 0.174;
	#endif

	yarp::dev::Pid * pid = new yarp::dev::Pid;
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			if(map(i,j) == k)
			{
				#ifdef ICUB5
					// old yarp definition
					JointPort[i].ipid->getPid(j,pid);
					pid->kp = int(kp * gain(map(i,j),0));
					pid->kd = int(kd * gain(map(i,j),1));
					pid->ki = int(ki * gain(map(i,j),2));
					JointPort[i].ipid->setPid(j,*pid);
				#elif defined(ICUBSIM) || defined(ICUB33)
					JointPort[i].ipid->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION,j,pid);
					pid->kp = kp * gain(map(i,j),0);
					pid->kd = kd * gain(map(i,j),1);
					pid->ki = ki * gain(map(i,j),2);
					JointPort[i].ipid->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION,j,*pid);
				#endif
			}
		}
	}
}

void Wrapper::getPidJoint(int k, double& kp, double& kd, double& ki)
{
	yarp::dev::Pid * pid = new yarp::dev::Pid;
	kp = 0; 
	kd = 0; 
	ki = 0;
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			if(map(i,j) == k)
			{
				#ifdef ICUB5
					// old yarp definition
					JointPort[i].ipid->getPid(j,pid);
				#elif defined(ICUBSIM) || defined(ICUB33)
					JointPort[i].ipid->getPid(yarp::dev::VOCAB_PIDTYPE_POSITION,j,pid);
				#endif
				kp = pid->kp;
				kd = pid->kd;
				ki = pid->ki;
				//printf("%2.2f, %2.2f, %2.2f \n", pid->kp, pid->kd, pid->ki);
			}
		}
	}
}

void Wrapper::getJointLimits(double * minPosition, double * maxPosition)
{
	for(int i=0; i<6; i++)
		for(int j=0; j<JointPort[i].number;j++)
			JointPort[i].ilimit->getLimits(j, &minPosition[int(map(i,j))], &maxPosition[int(map(i,j))]);
}

void Wrapper::readSensors(VectorXd &sens_pos, VectorXd &sens_vel, VectorXd &sens_tau)
{
	///////////////////////////////////////////////////////////////////////////////////////
	// read contact sensors
	for(int i=0; i<4; i++)
	{
		SensorPort[i].values = SensorPort[i].port.read();
		for(int j=0; j<3; j++)
		{
			FTsensor[i][0][j] = SensorPort[i].values->get(j).asDouble();
			FTsensor[i][1][j] = SensorPort[i].values->get(j+3).asDouble();
		}
		FTsensor[i][0][0] *= -1;
		FTsensor[i][1][0] *= -1;
	}

	///////////////////////////////////////////////////////////////////////////////////////
	// read IMU
	#if defined(ICUB5) || defined(ICUB33)
		Vector3d rot;
		rot[0]=EULER[0]; 
		rot[1]=EULER[1]; 
		rot[2]=EULER[2]; 
		linacc[0]=LINACC[0]; 
		linacc[1]=LINACC[1]; 
		linacc[2]=LINACC[2]; 
		angvel[0]=ANGVEL[0]; 
		angvel[1]=ANGVEL[1]; 
		angvel[2]=ANGVEL[2]; 

		// make rotation matrix
		Matrix3d X = rotmatrix(Vector3d(rot[0],0,0));
		Matrix3d Y = rotmatrix(Vector3d(0,rot[1],0));
		Matrix3d Z = rotmatrix(Vector3d(0,0,rot[2]));

		// robot gives roll/pitch/yaw
		R = Z * Y * X;

		// IMU mounting on the pelvis
		Matrix3d mount = rotmatrix(Vector3d(0,M_PI/2.0,0));

		// IMU mounting, set x axis pointing front, and z up
		R = rotmatrix(Vector3d(0,M_PI,0)) * R * rotmatrix(Vector3d(0,M_PI,0));

		// and then rotate 90 deg around y
		R = R * mount;

		// local-frame angular velocity
		angvel = mount * angvel;

		// local-frame acceleration ( gives normalized g when stationary )
		linacc = mount * linacc;
	#elif defined(ICUBSIM)
		SensorPort[4].values = SensorPort[4].port.read();
		Vector3d rot;
		for (int j=0; j<3; j++)
		{
			rot[j]	= SensorPort[4].values->get(0+j).asDouble() / 180.0 * M_PI;
			linacc[j] = SensorPort[4].values->get(3+j).asDouble();
			angvel[j] = SensorPort[4].values->get(6+j).asDouble() / 180.0 * M_PI;
		}

		// make rotation matrix
		// https://github.com/robotology/icub-gazebo/issues/18
		Matrix3d X = rotmatrix(Vector3d(rot[0],0,0));
		Matrix3d Y = rotmatrix(Vector3d(0,rot[1],0));
		Matrix3d Z = rotmatrix(Vector3d(0,0,rot[2]));

		// robot gives roll/pitch/yaw
		R = Z * Y * X;

		// IMU mounting, x and y directions point opposite in gazebo
		R = rotmatrix(Vector3d(0,0,M_PI)) * R * rotmatrix(Vector3d(0,0,-M_PI));
		angvel = rotmatrix(Vector3d(0,0,M_PI)) * angvel;
		linacc = rotmatrix(Vector3d(0,0,M_PI)) * linacc;
	#endif

	///////////////////////////////////////////////////////////////////////////////////////
	// read time
	double old_time = time;
	#if defined(ICUB5) || defined(ICUB33)
		time = yarp::os::Time::now() - startTime;
	#elif defined(ICUBSIM)
		SensorPort[5].values = SensorPort[5].port.read();
		if(SensorPort[5].values)
			time = SensorPort[5].values->get(0).asDouble() + SensorPort[5].values->get(1).asDouble()/pow(10,9) - startTime;
	#endif
	dt = max(time - old_time, minTimeStep);
	dt = min(dt, 0.02);

	///////////////////////////////////////////////////////////////////////////////////////
	// read joints
	for(int i=0; i<6; i++)
	{
		for(int j=0; j<JointPort[i].number;j++)
		{
			double position;
			JointPort[i].encs->getEncoder(j, &position);
			position *= M_PI / 180.0;
			sens_vel[map(i,j)+6] = (position - sens_pos[map(i,j)+6]) / dt;
			sens_pos[map(i,j)+6] = position;
			JointPort[i].itrq->getTorque(j, &sens_tau[map(i,j)+6]);
		}
	}
}

int Wrapper::applyExternalWrench(string link, VectorXd Force, double duration)
{
	#if defined(ICUBSIM)
		yarp::os::Bottle& bot = ExternalWrenchPort.port.prepare();
		bot.clear();
		bot.addString(link);
		bot.addFloat64(Force[0]);
		bot.addFloat64(Force[1]);
		bot.addFloat64(Force[2]);
		bot.addFloat64(Force[3]);
		bot.addFloat64(Force[4]);
		bot.addFloat64(Force[5]);
		bot.addFloat64(duration);
		ExternalWrenchPort.port.write();
	#endif
}

void Wrapper::rePID(bool walk)
{
	#if defined(ICUB5) || defined(ICUB33)
		for(int i=0;i<32;i++)
		{
			double mult = (i==8 || i==14) ? 0.2 : 1;
			setPidJoint(i,mult, mult, 0);
		}
	#elif defined(ICUBSIM)
		for(int i=0;i<32;i++)
			setPidJoint(i, 1, 1, 0);
		if(walk)
		{
			// hips
			setPidJoint(13-6, 10/17.453, 0.6/0.174, 0);
			setPidJoint(19-6, 10/17.453, 0.6/0.174, 0);
			setPidJoint(12-6, 10/17.453, 0.6/0.174, 0);
			setPidJoint(18-6, 10/17.453, 0.6/0.174, 0);
			
			// knees
			setPidJoint(15-6, 10/17.453, 0.3/0.174, 0);
			setPidJoint(21-6, 10/17.453, 0.3/0.174, 0);

			// ankles
			setPidJoint(14-6, 3/17.453, 0.3/0.174, 0);
			setPidJoint(20-6, 3/17.453, 0.3/0.174, 0);
			setPidJoint(16-6, 3/17.453, 0.1/0.174, 0);
			setPidJoint(22-6, 3/17.453, 0.1/0.174, 0);
			setPidJoint(17-6, 3/17.453, 0.1/0.174, 0);
			setPidJoint(23-6, 3/17.453, 0.1/0.174, 0);
		}
	#endif
}

void Wrapper::graspLeft(bool close)
{
	int released_targets[9] = {15, 30, 5, 97, 0, 0, 8, 0, 0}; 
	int grasped_targets[9] = {15, 90, 45, 97, 90, 110, 90, 110, 190}; 

	if(close)
		for(int j=7; j<16;j++)
			JointPort[4].pos->positionMove(j, grasped_targets[j-7]);
	else
		for(int j=7; j<16;j++)
			JointPort[4].pos->positionMove(j, released_targets[j-7]);
}

void Wrapper::graspRight(bool close)
{
	int released_targets[9] = {15, 30, 5, 0, 9, 0, 15, 5, 267}; 
	int grasped_targets[9] = {15, 30, 10, 145, 9, 0, 77, 50, 267}; 

	if(close)
		for(int j=7; j<16;j++)
			JointPort[5].pos->positionMove(j, grasped_targets[j-7]);
	else
		for(int j=7; j<16;j++)
			JointPort[5].pos->positionMove(j, released_targets[j-7]);
}