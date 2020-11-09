#pragma once

#include <stdio.h>
#include <unistd.h>
#include <iostream>

extern "C"
{
	#include "mip_sdk.h"
	#include "mip_sdk_interface.h"
	#include "byteswap_utilities.h"
	#include "mip_gx5_imu.h"
	#include "mip_gx5_25.h"
}

#define MIP_SDK_GX5_25_IMU_STANDARD_MODE  0x01
#define MIP_SDK_GX5_25_IMU_DIRECT_MODE	  0x02
#define NUM_COMMAND_LINE_ARGUMENTS 3
#define DEFAULT_PACKET_TIMEOUT_MS  1000 //milliseconds
//#define Sleep(x) usleep(x*1000.0)

//MIP Parser Packet Callback Functions
void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);
void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type);

class IMU
{
public:
	
	bool EKF;
	IMU();
	~IMU();
	int init(u32 com_port, u32 baudrate);
	void update(float euler[3], float linacc[3], float angvel[3]);

	//The primary device interface structure
	mip_interface device_interface;

	//Packet Counters (valid, timeout, and checksum errors)
	u32 filter_valid_packet_count;
	u32 ahrs_valid_packet_count;
	u32 filter_timeout_packet_count;
	u32 ahrs_timeout_packet_count;
	u32 filter_checksum_error_packet_count;
	u32 ahrs_checksum_error_packet_count;

	//AHRS
	mip_ahrs_euler_angles curr_ahrs_angles;
	mip_ahrs_scaled_accel curr_ahrs_accel;
	mip_ahrs_scaled_gyro  curr_ahrs_gyro;

	//FILTER
	mip_filter_attitude_euler_angles curr_filter_angles;
	mip_filter_linear_acceleration curr_filter_accel;
	mip_filter_compensated_angular_rate curr_filter_gyro;
};
