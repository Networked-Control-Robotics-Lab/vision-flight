#include <stdio.h>
#include <stdint.h>
#include <thread>
#include "trajectory_mission.hpp"
#include "sys_time.hpp"

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
}

#define GROUND_STATION_ID 0

void TrajectoryManager::send_mavlink_msg_to_serial(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	serial_puts((char *)buf, len);
}

void TrajectoryManager::add(trajectory_t& x, trajectory_t& y, trajectory_t& z)
{
	trajectory3d_t traj3d;
	traj3d.x = x;
	traj3d.y = y;
	traj3d.z = z;
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

void TrajectoryManager::print_trajectory(float *traj_coeff, int coeff_size)
{
	for(int i = 0; i < coeff_size; i++) {
		if(i == (coeff_size - 1)) {
			printf("%f\n\r", traj_coeff[i]);
		} else {
			printf("%f, ", traj_coeff[i]);
		}
	}
}

void TrajectoryManager::print_list()
{
	for(int i = 0; i < this->trajs.size(); i++) {
		/* position */
		printf("trajectory #%d:\n\r"
                       "position coefficients:\n\r"
		       "x: ", i);
		print_trajectory(this->trajs.at(i).x.pos_coeff, 8);
		printf("y: ");
		print_trajectory(this->trajs.at(i).y.pos_coeff, 8);
		printf("z: ");
		print_trajectory(this->trajs.at(i).z.pos_coeff, 8);

		/* velocity */
		printf("velocity coefficients:\n\r"
                       "x: ");
		print_trajectory(this->trajs.at(i).x.vel_coeff, 7);
		printf("y: ");
		print_trajectory(this->trajs.at(i).y.vel_coeff, 7);
		printf("z: ");
		print_trajectory(this->trajs.at(i).z.vel_coeff, 7);

		/* acceleration */
		printf("acceleration coefficients:\n\r"
                       "x: ");
		print_trajectory(this->trajs.at(i).x.accel_coeff, 6);
		printf("y: ");
		print_trajectory(this->trajs.at(i).y.accel_coeff, 6);
		printf("z: ");
		print_trajectory(this->trajs.at(i).z.accel_coeff, 6);

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
	int trial = 10;
	do {
		printf("mavlink: send polynomial trajectory write message. (handshacking)\n\r");

		uint8_t _z_enabled = (z_enabled == true ? 1 : 0);
		uint8_t _yaw_enabled = (yaw_enabled == true ? 1 : 0);

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

bool TrajectoryManager::send()
{
	if(this->trajs.size() == 0) {
		return true;
	}

	bool succeed;

	/* create mavlink message reception thread */
	std::thread thread_mavlink_rx(&TrajectoryManager::mavlink_rx_thread_entry, this);

	/* handshake step */
	succeed = send_traj_write_and_wait_ack();

	/* waypoints sending step */

	/* stop and kill the thread */
	this->stop_mavlink_rx_thread = true;
	thread_mavlink_rx.join();

	return succeed;

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

void TrajectoryManager::mavlink_rx_thread_entry()
{
	uint8_t recvd_msg = false;
	mavlink_message_t mavlink_recvd_msg;
	mavlink_status_t mavlink_rx_status;

	char c;
	while(this->stop_mavlink_rx_thread == false) {
		/* receive the message */
		if(serial_getc(&c) != -1) {
			//printf("%c", c);
			recvd_msg = mavlink_parse_char(MAVLINK_COMM_1, (uint8_t)c, &mavlink_recvd_msg, &mavlink_rx_status);

			if(recvd_msg == 1) {
				mavlink_rx_message_handler(mavlink_recvd_msg);
			}
		}
	}
	this->stop_mavlink_rx_thread = false;
}
