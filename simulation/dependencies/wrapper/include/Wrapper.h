#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>

#include "IMU.h"
#include "MedianFilter.h"
#include <pthread.h>
#include <sys/time.h>
#include <iostream>
#include <string>

#define minTimeStep 0.001

using namespace Eigen;
using namespace std;

struct sensorConnector
{
	yarp::os::BufferedPort<yarp::os::Bottle> port;
	yarp::os::Bottle *values;
};

struct jointConnector
{
	yarp::dev::PolyDriver			robotDevice;
	yarp::dev::IPositionDirect		*posdirect;
	yarp::dev::IPositionControl		*pos;
	yarp::dev::IEncoders			*encs;
	yarp::dev::IInteractionMode		*iint;
	yarp::dev::IImpedanceControl	*iimp;
	yarp::dev::ITorqueControl		*itrq;
	yarp::dev::IPidControl			*ipid;
	yarp::dev::IControlLimits		*ilimit;
	int								number;
#ifdef ICUB5
	// old yarp definition
	yarp::dev::IControlMode2		*ictrl;
#elif defined(ICUBSIM) || defined(ICUB33)
	yarp::dev::IControlMode			*ictrl;
#endif
};

class Wrapper
{
public:
	Wrapper();
	~Wrapper();

	// rotation algebra
	Matrix3d rotmatrix(Vector3d a);
	VectorXd quat_mul(VectorXd a, VectorXd b);
	VectorXd dc2quat(Matrix3d dircos);

	// YARP stuff
	yarp::os::Network yarp;
	yarp::os::Property params;
	string robotName;
	MatrixXd map;
	jointConnector JointPort[6];
	sensorConnector SensorPort[6];

	// setup functions
	int checkRobot(int argc, char *argv[]);
	int setupSensorConnector(sensorConnector &F, string moduleName);
	int setupJointConnector(jointConnector &C, string moduleName, int size);
	void initialize();
	void close();

	// measured variables
	Vector3d linacc;
	Vector3d angvel;
	MatrixXd R;
	Vector3d FTsensor[4][2];

	// time management
	double startTime;
	double time;
	double dt;
	
	// objects
	sensorConnector ObjectPorts[10];
	VectorXd Object[10];
	MedianFilter ObjectFilter[10];
	void initObject(int k, string name);
	VectorXd readObject(int k);

	// online functions
	void initializeJoint(VectorXd mode, VectorXd freeze);
	void controlJoint(VectorXd mode, VectorXd freeze, VectorXd ref_pos, VectorXd ref_tau);
	void setPidJoint(int k, double kp, double kd, double ki);
	void getPidJoint(int k, double& kp, double& kd, double& ki);
	void getJointLimits(double * minPosition, double * maxPosition);
	void readSensors(VectorXd &sens_pos, VectorXd &sens_vel, VectorXd &sens_tau);
	void graspLeft(bool close);
	void graspRight(bool close);

	// external wrenches
	sensorConnector ExternalWrenchPort;
	int applyExternalWrench(string link, VectorXd Force, double duration);	

	// change PID gains
	void rePID(bool walk);

	// IMU
	pthread_mutex_t mutex;
	IMU imu;
	float EULER[3];
	float LINACC[3];
	float ANGVEL[3];
};

