#include <string>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Int32.h"
#include "vins_mono_bridge.hpp"

#define DRONE_ID 1

void VINSMonoBridge::serial_open(string serial_port, int baudrate)
{
	//open the port
	this->serial_fd = open(serial_port.c_str(), O_RDWR | O_NOCTTY /*| O_NONBLOCK*/);

	if(this->serial_fd == -1) {
		ROS_FATAL("Failed to open the serial port.");
		exit(0);
	}

	//config the port
	struct termios options;

	tcgetattr(this->serial_fd, &options);

	options.c_cflag = CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	switch(baudrate) {
	case 9600:
		options.c_cflag |= B9600;
		break;
	case 57600:
		options.c_cflag |= B57600;
		break;
	case 115200:
		options.c_cflag |= B115200;
		break;
	default:
		ROS_FATAL("Unknown baudrate. try 9600, 57600, 115200 or check \"vins_mono_bridge.cpp\".");
		exit(0);
	}

	tcflush(this->serial_fd, TCIFLUSH);
	tcsetattr(this->serial_fd, TCSANOW, &options);
}

int VINSMonoBridge::serial_getc(char *c)
{
	return read(this->serial_fd, c, 1);
}

void VINSMonoBridge::serial_puts(char *s, size_t size)
{
	write(this->serial_fd, s, size);
}

uint8_t VINSMonoBridge::generate_checksum_byte(uint8_t init_val, uint8_t *payload, int payload_count)
{
	uint8_t result = init_val;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

void VINSMonoBridge::send_pose_to_serial(float* pos, float* vel, float* q)
{
	char msg_buf[VINS_MONO_SERIAL_MSG_SIZE] = {0};
	int msg_pos = 0;

	/* reserve 2 for start byte and checksum byte as header */
	//start byte
	msg_buf[msg_pos] = '@';
	msg_pos += sizeof(uint8_t);
	//checksum
	msg_buf[msg_pos] = 0;
	msg_pos += sizeof(uint8_t);
	//uav id
	msg_buf[msg_pos] = DRONE_ID;
	msg_pos += sizeof(uint8_t);

	/* pack payloads */
	//position (enu frame)
	memcpy(msg_buf + msg_pos, &pos[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &pos[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &pos[2], sizeof(float));
	msg_pos += sizeof(float);

	//velocity (enu frame)
	memcpy(msg_buf + msg_pos, &vel[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &vel[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &vel[2], sizeof(float));
	msg_pos += sizeof(float);

	//attitude quaternion (ned frame)
	memcpy(msg_buf + msg_pos, &q[0], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &q[1], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &q[2], sizeof(float));
	msg_pos += sizeof(float);
	memcpy(msg_buf + msg_pos, &q[3], sizeof(float));
	msg_pos += sizeof(float);

	//end byte
        msg_buf[msg_pos] = '+';
	msg_pos += sizeof(uint8_t);

	/* generate and fill the checksum field */
	msg_buf[1] = generate_checksum_byte(VINS_MONO_CHECKSUM_INIT_VAL, (uint8_t *)&msg_buf[3],
                                            VINS_MONO_SERIAL_MSG_SIZE - 4);

	serial_puts(msg_buf, VINS_MONO_SERIAL_MSG_SIZE);
}

void VINSMonoBridge::imu_msg_buf_push(uint8_t c)
{
	if(this->imu.buf_pos >= IMU_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < IMU_SERIAL_MSG_SIZE; i++) {
			this->imu.buf[i - 1] = this->imu.buf[i];
		}

		/* save new byte to the last array element */
		this->imu.buf[IMU_SERIAL_MSG_SIZE - 1] = c;
		this->imu.buf_pos = IMU_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		this->imu.buf[imu.buf_pos] = c;
		this->imu.buf_pos++;
	}
}

bool VINSMonoBridge::imu_message_decode(sensor_msgs::Imu& imu_msg)
{
	if((imu.buf[0]=='@') && (imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+')) {
		/* confirm checksum value */
		uint8_t recvd_checksum = this->imu.buf[1];
		uint8_t checksum = generate_checksum_byte(IMU_CHECKSUM_INIT_VAL, &this->imu.buf[3],
                                                          IMU_SERIAL_MSG_SIZE - 3);
		if(checksum != recvd_checksum) {
			/* checksum error */
			return false; 
		}

		/* decode package */
		memcpy(&imu.accel[0], &this->imu.buf[2], sizeof(float));
		memcpy(&imu.accel[1], &this->imu.buf[6], sizeof(float));
		memcpy(&imu.accel[2], &this->imu.buf[10], sizeof(float));
		memcpy(&imu.gyro[0], &this->imu.buf[14], sizeof(float));
		memcpy(&imu.gyro[1], &this->imu.buf[18], sizeof(float));
		memcpy(&imu.gyro[2], &this->imu.buf[22], sizeof(float));
	
		/* prepare ros message */
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.header.frame_id = "base_link";
		imu_msg.linear_acceleration.x = +imu.accel[1];
		imu_msg.linear_acceleration.y = +imu.accel[0];
		imu_msg.linear_acceleration.z = -imu.accel[2];
		imu_msg.angular_velocity.x = +imu.gyro[1] * M_PI / 180.0f;
		imu_msg.angular_velocity.y = +imu.gyro[0] * M_PI / 180.0f;
		imu_msg.angular_velocity.z = -imu.gyro[2] * M_PI / 180.0f;
		imu_msg.angular_velocity_covariance={1.2184696791468346e-07, 0.0, 0.0,
                                                     0.0, 1.2184696791468346e-07, 0.0,
                                                     0.0, 0.0, 1.2184696791468346e-07};
 		imu_msg.linear_acceleration_covariance={8.999999999999999e-08, 0.0, 0.0,
                                                        0.0, 8.999999999999999e-08, 0.0,
                                                        0.0, 0.0, 8.999999999999999e-08};
	}

	return true;
}

void VINSMonoBridge::launch_imu_message_listener()
{
	if(this->thread_imu_rx == nullptr) {
		this->thread_imu_rx = new std::thread(&VINSMonoBridge::serial_tx_thread_entry, std::ref(*this));
	}
}

void VINSMonoBridge::serial_tx_thread_entry()
{
	sensor_msgs::Imu imu_msg;
	ros::NodeHandle imu_node;
	ros::Publisher imu_publisher = imu_node.advertise<sensor_msgs::Imu>("imu/data_raw", 5);

	char c;

	while(this->kill_thread_signal == false) {
		if(serial_getc(&c) != -1) {
			if(imu_message_decode(imu_msg) == true) {
				imu_publisher.publish(imu_msg);
			}
		}
	}
}
