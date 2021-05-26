#include <stdio.h>
#include <stdint.h>
#include <thread>
#include <unistd.h>
#include "trajectory_mission.hpp"
#include "sys_time.hpp"
#include "mission_manager.hpp"

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
}

#define GROUND_STATION_ID 0

void TrajectoryManager::serial_puts(char *s, size_t size)
{
	write(this->serial_fd, s, size);
}

void TrajectoryManager::send_mavlink_msg_to_serial(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	serial_puts((char *)buf, len);
}

void TrajectoryManager::add(trajectory_t& x, trajectory_t& y, trajectory_t& z, float flight_time)
{
	trajectory3d_t traj3d;
	traj3d.x = x;
	traj3d.y = y;
	traj3d.z = z;
	traj3d.flight_time = flight_time;
	this->trajs.push_back(traj3d); 
}

void TrajectoryManager::clear()
{
	this->trajs.clear();
}

void TrajectoryManager::get_trajectory(int index, trajectory_t& x, trajectory_t& y, trajectory_t& z)
{
	x = this->trajs.at(index).x;
	y = this->trajs.at(index).y;
	z = this->trajs.at(index).z;
}

void TrajectoryManager::print_trajectory(float *coeff, int coeff_size)
{
	for(int i = 0; i < coeff_size; i++) {
		if(i == (coeff_size - 1)) {
			printf("%f\n\r", coeff[i]);
		} else {
			printf("%f, ", coeff[i]);
		}
	}
}

void TrajectoryManager::print_list()
{
	for(int i = 0; i < this->trajs.size(); i++) {
		/* position */
		printf("trajectory coefficients#%d:\n\r"
		       "x: ", i);
		print_trajectory(this->trajs.at(i).x.coeff, 8);
		printf("y: ");
		print_trajectory(this->trajs.at(i).y.coeff, 8);
		printf("z: ");
		print_trajectory(this->trajs.at(i).z.coeff, 8);

		printf("---\n\r");
	}
}

bool TrajectoryManager::wait_trajectory_ack()
{
	double start_time = get_sys_time_s();

	while(recvd_traj_ack == false) {
		double current_time = get_sys_time_s();
		double elapsed_time = current_time - start_time;

		/* one seconds timeout */
		if(elapsed_time >= 1.0f) {
			return false;
		}
	}

	this->recvd_traj_ack = false;
	return true;
}

bool TrajectoryManager::send_traj_write_and_wait_ack()
{
	int trial = RETRY_TIME_MAX;
	do {
		printf("mavlink: send polynomial trajectory write message. (handshacking)\n\r");

		uint8_t _z_enabled = (z_enabled == true ? 1 : 0);
		uint8_t _yaw_enabled = 0;

		mavlink_message_t msg;
		mavlink_msg_polynomial_trajectory_write_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id, 0,
                                                                  this->trajs.size(), _z_enabled, _yaw_enabled);
		send_mavlink_msg_to_serial(&msg);

		bool ack_recvd = wait_trajectory_ack();
		if(ack_recvd == true) {
			printf("succeeded.\n\r");
			return true;
		} else {
			printf("timeout!\n\r");
		}
	} while(--trial);

	return false;
}

bool TrajectoryManager::send_traj_item_and_wait_ack(uint8_t index, uint8_t type)
{
	float *traj_coeff;
	float flight_time = this->trajs.at(index).flight_time;

	string s;
	switch(type) {
	case TRAJECTORY_TYPE_X:
		s = "x";
		traj_coeff = this->trajs.at(index).x.coeff;
		break;
	case TRAJECTORY_TYPE_Y:
		s = "y";
		traj_coeff = this->trajs.at(index).y.coeff;
		break;
	case TRAJECTORY_TYPE_Z:
		s = "z";
		traj_coeff = this->trajs.at(index).z.coeff;
		break;
	}

	int trial = RETRY_TIME_MAX;
	do {
		printf("mavlink: [#%d] send %s trajectory.\n\r", index, s.c_str());

		mavlink_message_t msg;
		mavlink_msg_polynomial_trajectory_item_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id,
                                                                 0, type, index, traj_coeff, flight_time);
		send_mavlink_msg_to_serial(&msg);

		bool ack_recvd = wait_trajectory_ack();
		if(ack_recvd == true) {
			printf("succeeded.\n\r");
			return true;
		} else {
			printf("timeout!\n\r");
		}
	} while(--trial);

	printf("polynomial trajectory item sending failed.\n\r");

	return false;
}

bool TrajectoryManager::send_mission()
{
	if(this->trajs.size() == 0) {
		return true;
	}

	bool succeed = true;

	/* handshake step */
	if(send_traj_write_and_wait_ack() == false) {
		return false;
	}
	
	/* trajectories sending step */
	for(int i = 0; i < this->trajs.size(); i++) {
		succeed = send_traj_item_and_wait_ack(i, TRAJECTORY_TYPE_X);
		if(succeed == false) {
			break;
		}

		succeed = send_traj_item_and_wait_ack(i, TRAJECTORY_TYPE_Y);
		if(succeed == false) {
			break;
		}

		if(this->z_enabled == true) {
			succeed = send_traj_item_and_wait_ack(i, TRAJECTORY_TYPE_Z);
			if(succeed == false) {
				break;
			}		
		}
	}

	return succeed;
}

void TrajectoryManager::send_takeoff_cmd()
{
	uint8_t confirm = 0;
	float params[7] = {0};

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id, 0,
	                                   MAV_CMD_NAV_TAKEOFF, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}

void TrajectoryManager::send_land_cmd()
{
	uint8_t confirm = 0;
	float params[7];

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id, 0,
	                                   MAV_CMD_NAV_LAND, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}

void TrajectoryManager::start(bool looping)
{
	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_cmd_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id,
                                                        0, TRAJECTORY_FOLLOWING_START, looping);
	send_mavlink_msg_to_serial(&msg);
}

void TrajectoryManager::stop()
{
	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_cmd_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id,
                                                        0, TRAJECTORY_FOLLOWING_STOP, 0);
	send_mavlink_msg_to_serial(&msg);
}

void TrajectoryManager::mavlink_rx_message_handler(mavlink_message_t& msg)
{
	switch(msg.msgid) {
	/* POLYNOMIAL_TRAJECTORY_ACK */
	case 11002: {
		mavlink_polynomial_trajectory_ack_t traj_ack;
		mavlink_msg_polynomial_trajectory_ack_decode(&msg, &traj_ack);
		this->traj_ack_val = traj_ack.ack_val;
		this->recvd_traj_ack = true;
		break;
	}
	}
}
