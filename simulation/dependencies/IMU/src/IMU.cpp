#include "IMU.h"

IMU * imu_pointer;

IMU::IMU()
{
}

IMU::~IMU()
{
}

int IMU::init(u32 com_port, u32 baudrate)
{
	imu_pointer = this;
	EKF = false;

	// Settings
	u8  temp_string[20] = {0};
	u8  enable = 1;
	u8  data_stream_format_descriptors[10];
	u16 data_stream_format_decimation[10];
	u8  data_stream_format_num_entries = 0;
	u16 device_descriptors_size  = 128*2;
	u8  com_mode = 0;
	float angles[3]             = {0};

	filter_valid_packet_count = 0;
	ahrs_valid_packet_count = 0;
	filter_timeout_packet_count = 0;
	ahrs_timeout_packet_count = 0;
	filter_checksum_error_packet_count = 0;
	ahrs_checksum_error_packet_count = 0;



	//Initialize the interface to the device
	if(mip_interface_init(com_port, baudrate, &device_interface, DEFAULT_PACKET_TIMEOUT_MS) != MIP_INTERFACE_OK)
	return -1;


	// Putting Device Into Standard Mode
	device_descriptors_size  = 128*2;
	com_mode = MIP_SDK_GX5_25_IMU_STANDARD_MODE;

	//Set communication mode
	while(mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_WRITE, &com_mode) != MIP_INTERFACE_OK){}

	//Verify device mode setting
	while(mip_system_com_mode(&device_interface, MIP_FUNCTION_SELECTOR_READ, &com_mode) != MIP_INTERFACE_OK){}

	if(com_mode != MIP_SDK_GX5_25_IMU_STANDARD_MODE)
		printf("ERROR: Standard mode not established\n\n");
	else
	{
		/*
		// Put the GX5-25 into idle mode
		while(mip_base_cmd_idle(&device_interface) != MIP_INTERFACE_OK){}

		// Try to ping the GX5-25
		while(mip_base_cmd_ping(&device_interface) != MIP_INTERFACE_OK){}

		// Reset the filter
		while(mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK){}

		// Initialize the filter with Euler Angles
		angles[0] = angles[1] = angles[2] = 0;
		while(mip_filter_set_init_attitude(&device_interface, angles) != MIP_INTERFACE_OK){}

		// Reset the filter
		while(mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK){}

		// Initialize the filter with a heading
		while(mip_filter_set_init_heading(&device_interface, angles[0]) != MIP_INTERFACE_OK){}

		// Reset the filter
		while(mip_filter_reset_filter(&device_interface) != MIP_INTERFACE_OK){}
		*/


		// Setup the GX5-25 dataset callbacks
		if(mip_interface_add_descriptor_set_callback(&device_interface, MIP_FILTER_DATA_SET, NULL, &filter_packet_callback) != MIP_INTERFACE_OK)
		return -1;
		if(mip_interface_add_descriptor_set_callback(&device_interface, MIP_AHRS_DATA_SET, NULL, &ahrs_packet_callback) != MIP_INTERFACE_OK)
		return -1;

		if(EKF)
		{
			// Setup the FILTER datastream format
			data_stream_format_descriptors[0] = MIP_FILTER_DATA_ATT_EULER_ANGLES;
			data_stream_format_descriptors[1] = MIP_FILTER_DATA_LINEAR_ACCELERATION;
			data_stream_format_descriptors[2] = MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE;
			data_stream_format_decimation[0]  = 0x01;
			data_stream_format_decimation[1]  = 0x01;
			data_stream_format_decimation[2]  = 0x01;
			data_stream_format_num_entries = 3;
			while(mip_3dm_cmd_filter_message_format(&device_interface, 
													MIP_FUNCTION_SELECTOR_WRITE, 
													&data_stream_format_num_entries,
													data_stream_format_descriptors, 
													data_stream_format_decimation) != MIP_INTERFACE_OK) {}

			// Poll the Estimation Filter data
			while(mip_3dm_cmd_poll_filter(  &device_interface, 
											MIP_3DM_POLLING_ENABLE_ACK_NACK, 
											data_stream_format_num_entries, 
											data_stream_format_descriptors) != MIP_INTERFACE_OK){}

			// Enable the FILTER datastream
			while(mip_3dm_cmd_continuous_data_stream(&device_interface, 
													 MIP_FUNCTION_SELECTOR_WRITE, 
													 MIP_3DM_INS_DATASTREAM, 
													 &enable) != MIP_INTERFACE_OK){}
		}
		else
		{
			// Setup the AHRS datastream format
			data_stream_format_descriptors[0] = MIP_AHRS_DATA_ACCEL_SCALED;
			data_stream_format_descriptors[1] = MIP_AHRS_DATA_GYRO_SCALED;
			data_stream_format_descriptors[2] = MIP_AHRS_DATA_EULER_ANGLES;
			data_stream_format_decimation[0]  = 0x01;
			data_stream_format_decimation[1]  = 0x01;
			data_stream_format_decimation[2]  = 0x01;
			data_stream_format_num_entries = 3;
			while(mip_3dm_cmd_ahrs_message_format(&device_interface, 
												  MIP_FUNCTION_SELECTOR_WRITE, 
												  &data_stream_format_num_entries,
												  data_stream_format_descriptors, 
												  data_stream_format_decimation) != MIP_INTERFACE_OK){}

			// Poll the AHRS data
			while(mip_3dm_cmd_poll_ahrs(&device_interface, 
										MIP_3DM_POLLING_ENABLE_ACK_NACK, 
										data_stream_format_num_entries, 
										data_stream_format_descriptors) != MIP_INTERFACE_OK){}

			 // Enable the AHRS datastream
			while(mip_3dm_cmd_continuous_data_stream(&device_interface, 
													 MIP_FUNCTION_SELECTOR_WRITE, 
													 MIP_3DM_AHRS_DATASTREAM, 
													 &enable) != MIP_INTERFACE_OK){}
		
		}
	}

}



void IMU::update(float euler[3], float linacc[3], float angvel[3])
{
	//Update the parser (this function reads the port and parses the bytes
	mip_interface_update(&device_interface);
	if(EKF)
	{
		euler[0] = curr_filter_angles.roll;
		euler[1] = curr_filter_angles.pitch;
		euler[2] = curr_filter_angles.yaw;
		linacc[0] = curr_filter_accel.x;
		linacc[1] = curr_filter_accel.y;
		linacc[2] = curr_filter_accel.z;
		angvel[0] = curr_filter_gyro.x;
		angvel[1] = curr_filter_gyro.y;
		angvel[2] = curr_filter_gyro.z;
	}
	else
	{
		euler[0] = curr_ahrs_angles.roll;
		euler[1] = curr_ahrs_angles.pitch;
		euler[2] = curr_ahrs_angles.yaw;
		linacc[0] = curr_ahrs_accel.scaled_accel[0];
		linacc[1] = curr_ahrs_accel.scaled_accel[1];
		linacc[2] = curr_ahrs_accel.scaled_accel[2];
		angvel[0] = curr_ahrs_gyro.scaled_gyro[0];
		angvel[1] = curr_ahrs_gyro.scaled_gyro[1];
		angvel[2] = curr_ahrs_gyro.scaled_gyro[2];
	}
}















void filter_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 mip_field_header *field_header;
 u8               *field_data;
 u16              field_offset = 0;

 //The packet callback can have several types, process them all
 switch(callback_type)
 {
  ///
  //Handle valid packets
  ///

  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
  {
   imu_pointer->filter_valid_packet_count++;

   ///
   //Loop through all of the data fields
   ///

   while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
   {

    ///
    // Decode the field
    ///

    switch(field_header->descriptor)
    {
     ///
     // Estimated Attitude, Euler Angles
     ///

     case MIP_FILTER_DATA_ATT_EULER_ANGLES:
     {
      memcpy(&imu_pointer->curr_filter_angles, field_data, sizeof(mip_filter_attitude_euler_angles));

      //For little-endian targets, byteswap the data field
      mip_filter_attitude_euler_angles_byteswap(&imu_pointer->curr_filter_angles);

     }break;
	 case MIP_FILTER_DATA_LINEAR_ACCELERATION:
     {
      memcpy(&imu_pointer->curr_filter_accel, field_data, sizeof(mip_filter_linear_acceleration));

      //For little-endian targets, byteswap the data field
      mip_filter_linear_acceleration_byteswap(&imu_pointer->curr_filter_accel);

     }break;
	 case MIP_FILTER_DATA_COMPENSATED_ANGULAR_RATE:
     {
      memcpy(&imu_pointer->curr_filter_gyro, field_data, sizeof(mip_filter_compensated_angular_rate));

      //For little-endian targets, byteswap the data field
      mip_filter_compensated_angular_rate_byteswap(&imu_pointer->curr_filter_gyro);

     }break;

     default: break;
    }
   }
  }break;


  ///
  //Handle checksum error packets
  ///

  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
  {
   imu_pointer->filter_checksum_error_packet_count++;
  }break;

  ///
  //Handle timeout packets
  ///

  case MIP_INTERFACE_CALLBACK_TIMEOUT:
  {
   imu_pointer->filter_timeout_packet_count++;
  }break;
  default: break;
 }

}

void ahrs_packet_callback(void *user_ptr, u8 *packet, u16 packet_size, u8 callback_type)
{
 mip_field_header *field_header;
 u8               *field_data;
 u16              field_offset = 0;

 //The packet callback can have several types, process them all
 switch(callback_type)
 {
  ///
  //Handle valid packets
  ///

  case MIP_INTERFACE_CALLBACK_VALID_PACKET:
  {
   imu_pointer->ahrs_valid_packet_count++;

   ///
   //Loop through all of the data fields
   ///

   while(mip_get_next_field(packet, &field_header, &field_data, &field_offset) == MIP_OK)
   {

    ///
    // Decode the field
    ///

    switch(field_header->descriptor)
    {
     ///
     // Scaled Accelerometer
     ///

     case MIP_AHRS_DATA_ACCEL_SCALED:
     {
      memcpy(&imu_pointer->curr_ahrs_accel, field_data, sizeof(mip_ahrs_scaled_accel));

      //For little-endian targets, byteswap the data field
      mip_ahrs_scaled_accel_byteswap(&imu_pointer->curr_ahrs_accel);

     }break;

     ///
     // Scaled Gyro
     ///

     case MIP_AHRS_DATA_GYRO_SCALED:
     {
      memcpy(&imu_pointer->curr_ahrs_gyro, field_data, sizeof(mip_ahrs_scaled_gyro));

      //For little-endian targets, byteswap the data field
      mip_ahrs_scaled_gyro_byteswap(&imu_pointer->curr_ahrs_gyro);

     }break;

     ///
     // Euler
     ///

     case MIP_AHRS_DATA_EULER_ANGLES:
     {
      memcpy(&imu_pointer->curr_ahrs_angles, field_data, sizeof(mip_ahrs_euler_angles));

      //For little-endian targets, byteswap the data field
      mip_ahrs_euler_angles_byteswap(&imu_pointer->curr_ahrs_angles);

     }break;

     default: break;
    }
   }
  }break;

  ///
  //Handle checksum error packets
  ///

  case MIP_INTERFACE_CALLBACK_CHECKSUM_ERROR:
  {
   imu_pointer->ahrs_checksum_error_packet_count++;
  }break;

  ///
  //Handle timeout packets
  ///

  case MIP_INTERFACE_CALLBACK_TIMEOUT:
  {
   imu_pointer->ahrs_timeout_packet_count++;
  }break;
  default: break;
 }

}



