#include <stdio.h>
#include <vector>
#include <thread>
#include <unistd.h>
#include "waypoint_mission.hpp"
#include "sys_time.hpp"
#include "mission_manager.hpp"

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
}

#define GROUND_STATION_ID 0

using namespace std;

void WaypointManager::serial_puts(char *s, size_t size)
{
	write(this->serial_fd, s, size);
}

void WaypointManager::send_mavlink_msg_to_serial(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	serial_puts((char *)buf, len);
}

void WaypointManager::add(float x, float y, float z)
{
	waypoint_t new_waypoint;
	new_waypoint.x = x;
	new_waypoint.y = y;
	new_waypoint.z = z;

	this->waypoints.push_back(new_waypoint);
}

int WaypointManager::size()
{
	return this->waypoints.size();
}

void WaypointManager::clear()
{
	this->waypoints.clear();
}

void WaypointManager::get_waypoint(int index, waypoint_t& waypoint)
{
	waypoint = waypoints.at(index);
}

void WaypointManager::print_list()
{
	vector<waypoint_t>::iterator it = this->waypoints.begin();

	for(int i = 0; i < this->waypoints.size(); i++) {
		printf("#%d: x = %fm, y = %fm, z = %fm\n\r", i, it->x, it->y, it->z);
		it++;
	}
}

bool WaypointManager::wait_mission_request_int()
{
	double start_time = get_sys_time_s();

	while(1) {
		double current_time = get_sys_time_s();
		double elapsed_time = current_time - start_time;

		if(this->recvd_mission_request_int == true) {
			this->recvd_mission_request_int = false;
			return true;
		}

		/* one seconds timeout */
		if(elapsed_time >= 1.0f) {
			this->recvd_mission_request_int = false;
			return false;
		}
	}
}

bool WaypointManager::wait_mission_ack()
{
	double start_time = get_sys_time_s();

	while(1) {
		double current_time = get_sys_time_s();
		double elapsed_time = current_time - start_time;

		if(this->recvd_mission_ack == true) {
			this->recvd_mission_ack = false;
			return true;
		}

		/* one seconds timeout */
		if(elapsed_time >= 1.0f) {
			this->recvd_mission_ack = false;
			return false;
		}
	}
}

bool WaypointManager::wait_command_long_ack()
{
	double start_time = get_sys_time_s();

	while(1) {
		double current_time = get_sys_time_s();
		double elapsed_time = current_time - start_time;

		if(this->recvd_cmd_long_ack == true) {
			this->recvd_cmd_long_ack = false;
			return true;
		}

		/* one seconds timeout */
		if(elapsed_time >= 1.0f) {
			this->recvd_cmd_long_ack = false;
			return false;
		}
	}
}

bool WaypointManager::send_mission_count_and_wait_ack()
{
	int trial = RETRY_TIME_MAX;
	do {
		printf("mavlink: send mission count message. (handshacking)\n\r");

		mavlink_message_t msg;
		mavlink_msg_mission_count_pack(GROUND_STATION_ID, 1, &msg, this->target_id, 0,
		                               this->waypoints.size(), MAV_MISSION_TYPE_MISSION);
		send_mavlink_msg_to_serial(&msg);

		bool ack_recvd = wait_mission_request_int();
		if(ack_recvd == true) {
			printf("succeeded.\n\r");
			return true;
		} else {
			printf("timeout!\n\r");
		}
	} while(--trial);

	return false;
}

bool WaypointManager::send_mission_waypoint(int index, bool is_last_waypoint)
{
	uint8_t _frame = MAV_FRAME_LOCAL_NED;
	switch(this->frame) {
	case WAYPOINT_GEODETIC_FRAME:
		_frame = MAV_FRAME_GLOBAL;
		break;
	case WAYPOINT_CARTESIAN_FRAME:
		_frame = MAV_FRAME_LOCAL_ENU;
		break;
	default:
		return false;
	}

	uint16_t command = 0;
	uint8_t current = 0;
	uint8_t autocontinue = 0;
	float params[4] = {0.0f};

	waypoint_t waypoint;
	get_waypoint(index, waypoint);

	bool recvd_ack = false;
	int trial = RETRY_TIME_MAX;
	do {
		/* send requested waypoint */
		printf("mavlink: send waypoint #%d\n\r", index);

		mavlink_message_t msg;
		mavlink_msg_mission_item_int_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id, 0,
		                                       index, _frame, command, current, autocontinue,
		                                       params[0], params[1], params[2], params[3],
		                                       waypoint.x, waypoint.y, waypoint.z,
		                                       MAV_MISSION_TYPE_MISSION);
		send_mavlink_msg_to_serial(&msg);

		bool ack_recvd = false;
		if(is_last_waypoint == true) {
			/* last waypoint: should receive mission ack message */
			ack_recvd = wait_mission_ack();
		} else {
			/* not the last waypoint: should receive mission request int message */
			ack_recvd = wait_mission_request_int();
		}


		/* check if successfully received the ack */
		if(ack_recvd == true) {
			return true;
		} else {
			printf("timeout.\n\r");
			if(trial == 0) {
				return false;
			}
		}
	} while(--trial);
}

bool WaypointManager::send_mission()
{
	if(this->waypoints.size() == 0) {
		return true;
	}

	/* handshake step */
	if(send_mission_count_and_wait_ack() == false) {
		return false;
	}

	/* waypoints sending step */
	bool succeed = true;
	while(this->recvd_mission_ack == false) {
		bool is_last_waypoint = (this->mission_request_sequence >= (waypoints.size() - 1));

		succeed = send_mission_waypoint(this->mission_request_sequence, is_last_waypoint);

		if(is_last_waypoint == true && succeed == true) {
			break;
		}

		if(succeed == false) {
			break;
		}
	}

	return succeed;
}

void WaypointManager::send_auto_mode_switch_cmd()
{
	//XXX: emulate what qgroundcontrol do, may change the behavior later
	uint8_t base_mode = 81;
	uint32_t custom_mode = 0;

	mavlink_message_t msg;
	mavlink_msg_set_mode_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg,
	                               this->target_id, base_mode, custom_mode);
	send_mavlink_msg_to_serial(&msg);
}

bool WaypointManager::send_command_long_message(const char* prompt, uint16_t cmd, float* params)
{
	uint8_t confirm = 1;
	mavlink_message_t msg;

	int trial = RETRY_TIME_MAX;
	do {
		printf("%s", prompt);

		mavlink_msg_command_long_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id, 0,
		                                   MAV_CMD_MISSION_START, confirm, params[0], params[1], params[2], params[3],
		                                   params[4], params[5], params[6]);
		send_mavlink_msg_to_serial(&msg);

		bool ack_recvd = wait_command_long_ack();
		if(ack_recvd == true) {
			if(this->cmd_ack_result == MAV_RESULT_ACCEPTED) {
				printf("command accepted.\n\r");
			} else if(this->cmd_ack_result == MAV_RESULT_DENIED) {
				printf("command denied.\n\r");
			} else {
				printf("error, unknown ack status by the uav.\n\r");
			}

			return true;
		} else {
			printf("timeout!\n\r");
		}
	} while(--trial);

	return false;
}

bool WaypointManager::send_start_cmd()
{
	if(this->size() <= 0) {
		printf("abort, waypoint list is empty.\n\r");
		return false;
	}

	float params[7] = {0};
	params[0] = 0;                //start waypoint number
	params[1] = this->size() - 1; //end waypoint number

	return send_command_long_message("mavlink: send mission start command.\n\r",
	                                 MAV_CMD_MISSION_START, params);
}

bool WaypointManager::send_takeoff_cmd()
{
	float params[7] = {0};

	return send_command_long_message("mavlink: send takeoff command.\n\r",
	                                 MAV_CMD_NAV_TAKEOFF, params);
}

bool WaypointManager::send_land_cmd()
{
	float params[7] = {0};

	return send_command_long_message("mavlink: send landing command.\n\r",
	                                 MAV_CMD_NAV_LAND, params);
}

bool WaypointManager::send_halt_cmd()
{
	float params[7] = {0};
	params[0] = MAV_GOTO_DO_HOLD;
	params[1] = MAV_GOTO_HOLD_AT_CURRENT_POSITION;
	params[2] = MAV_FRAME_LOCAL_ENU;

	return send_command_long_message("mavlink: send halt command.\n\r",
	                                 MAV_CMD_OVERRIDE_GOTO, params);
}

bool WaypointManager::send_resume_cmd()
{
	float params[7] = {0};
	params[0] = MAV_GOTO_DO_CONTINUE;

	return send_command_long_message("mavlink: send resume command.\n\r",
	                                 MAV_CMD_OVERRIDE_GOTO, params);
}

bool WaypointManager::send_goto_cmd(float yaw, float x, float y, float z)
{
	float params[7] = {0};
	params[0] = MAV_GOTO_DO_HOLD;
	params[1] = MAV_GOTO_HOLD_AT_SPECIFIED_POSITION;
	params[2] = MAV_FRAME_LOCAL_ENU;
	params[3] = yaw;
	params[4] = x;
	params[5] = y;
	params[6] = z;

	return send_command_long_message("mavlink: send goto command.\n\r",
	                                 MAV_CMD_OVERRIDE_GOTO, params);
}

void WaypointManager::mavlink_rx_message_handler(mavlink_message_t& msg)
{
	switch(msg.msgid) {
	/* MISSION_REQUEST_INT */
	case 51:
		mavlink_mission_request_int_t mission_item;
		mavlink_msg_mission_request_int_decode(&msg, &mission_item);
		this->recvd_mission_request_int = true;
		this->mission_request_sequence = mission_item.seq;
		break;
	/* MISSION_ACK */
	case 47:
		this->recvd_mission_ack = true;
		break;
	/* COMMAND_ACK */
	case 77:
		this->recvd_cmd_long_ack = true;
		mavlink_command_ack_t command_ack_item;
		mavlink_msg_command_ack_decode(&msg, &command_ack_item);
		this->cmd_ack_result = command_ack_item.result;
		break;
	}
}
