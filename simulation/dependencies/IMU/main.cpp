#include "IMU.h"
#include <sys/time.h>

//Help Functions
void print_command_line_usage()
{
	printf("\n\n");
	printf("Usage:\n");
	printf("-----------------------------------------------------------------------\n\n");

	printf("   ./main [com_port_num] [baudrate]\n");
	printf("\n\n");
	printf("   Example: \"./main 0 115200\", Opens a connection to the \n");
	printf("             ./main on COM0, with a baudrate of 115200.\n");
	printf("\n\n");
	printf("   [ ] - required command input.\n");
	printf("\n-----------------------------------------------------------------------\n");
	printf("\n\n");
}

int main(int argc, char* argv[])
{
	printf("Check which tty is associated to the IMU:\n");
	printf("ls /dev/ttyACM (now press tab)\n");
	printf("sudo chmod 666 /dev/ttyACM0\n");
	printf("sudo adduser #user dialout\n");
	
	// Verify the command line arguments
	if(argc != NUM_COMMAND_LINE_ARGUMENTS)
	{
		print_command_line_usage();
		return -1;
	}

	//Convert the arguments
	u32 com_port, baudrate;
	com_port = atoi(argv[1]);
	baudrate = atoi(argv[2]);

	IMU imu;
	imu.init(com_port, baudrate);

	timeval start,end;
	gettimeofday(&start, NULL);

	// Wait for packets to arrive//
	while(1)
	{
		float euler[3];
		float linacc[3];
		float angvel[3];
		imu.update(euler, linacc, angvel);
		mip_interface_update(&imu.device_interface);
		printf("%3.3f, %3.3f, %3.3f  |||||| ", euler[0], euler[1], euler[2]);
		printf("%3.3f, %3.3f, %3.3f  |||||| ", angvel[0], angvel[1], angvel[2]);
		printf("%3.3f, %3.3f, %3.3f  |||||| ", linacc[0], linacc[1], linacc[2]);
		printf("\n");

		gettimeofday(&end, NULL);
		printf("%f\n", (end.tv_sec-start.tv_sec)*1000 + (end.tv_usec-start.tv_usec)/1000.0);
		Sleep(1);
	}
}


