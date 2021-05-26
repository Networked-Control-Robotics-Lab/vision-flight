#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <string>
#include <thread>
#include "ros/ros.h"
#include "mission_manager.hpp"

using namespace std;

int MissionManager::serial_getc(char *c)
{
	return read(this->serial_fd, c, 1);
}

void MissionManager::open_serial_port(string& port_name, int baudrate)
{
	//open the port
	this->serial_fd = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

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
		ROS_FATAL("Unknown baudrate. try 9600, 57600, 115200 or check \"mission_manager.cpp\".");
		exit(0);
	}

	tcflush(this->serial_fd, TCIFLUSH);
	tcsetattr(this->serial_fd, TCSANOW, &options);
}

MissionManager::MissionManager(int target_id, string serial_port, int baudrate,
                               int frame, bool traj_z_enabled): thread_mavlink_rx(nullptr)
{
	open_serial_port(serial_port, baudrate);

	/* intialize waypoint manager and trajectory manager */
	this->waypoint = WaypointManager(target_id, frame, this->serial_fd);
	this->trajectory = TrajectoryManager(target_id, traj_z_enabled, this->serial_fd);
}

void MissionManager::mavlink_rx_thread_entry()
{
	uint8_t recvd_msg = false;
	mavlink_message_t mavlink_recvd_msg;
	mavlink_status_t mavlink_rx_status;

	char c;

	while(1) {
		/* receive the message */
		if(serial_getc(&c) != -1) {
			//printf("%c", c);
			recvd_msg = mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)c, &mavlink_recvd_msg, &mavlink_rx_status);

			if(recvd_msg == 1) {
				this->waypoint.mavlink_rx_message_handler(mavlink_recvd_msg);
				this->trajectory.mavlink_rx_message_handler(mavlink_recvd_msg);
			}
		}
	}
}
