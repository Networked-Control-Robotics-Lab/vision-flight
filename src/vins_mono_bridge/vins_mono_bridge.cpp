#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include "ros/ros.h"
#include "vins_mono_bridge.hpp"

#define VINS_MONO_CHECKSUM_INIT_VAL 19
#define VINS_MONO_SERIAL_MSG_SIZE 44
#define DRONE_ID 1

VINSMonoBridge::VINSMonoBridge(string serial_port, int baudrate)
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
		ROS_FATAL("Unknown baudrate. try 9600, 57600, 115200 or check \"serial.cpp\".");
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

uint8_t VINSMonoBridge::generate_vins_mono_checksum_byte(uint8_t *payload, int payload_count)
{
	uint8_t result = VINS_MONO_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

void VINSMonoBridge::send_pose_to_serial(float* pos, float* vel, float* q)
{
#if 0
	ROS_INFO("[%fHz], position=(x:%.2f, y:%.2f, z:%.2f), "
                 "orientation=(x:%.2f, y:%.2f, z:%.2f, w:%.2f),"
                 "velocity=(x:%.2f,y:%.2f,z:%.2f)",
                 real_freq, pos[0] * 100.0f, pos[1] * 100.0f, pos[2] * 100.0f,
                 q[1], q[2], q[3], q[0], vel[0], vel[1], vel[2]);
#endif

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
	msg_buf[1] = generate_vins_mono_checksum_byte((uint8_t *)&msg_buf[3], VINS_MONO_SERIAL_MSG_SIZE - 4);

	serial_puts(msg_buf, VINS_MONO_SERIAL_MSG_SIZE);
}

