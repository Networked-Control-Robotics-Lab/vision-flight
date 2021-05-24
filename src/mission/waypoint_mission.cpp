#include <stdio.h>
#include <vector>
#include <thread>
#include "serial.hpp"
#include "waypoint_mission.hpp"
#include "sys_time.hpp"

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
}

#define GROUND_STATION_ID 0

using namespace std;

void WaypointManager::send_mavlink_msg_to_serial(mavlink_message_t *msg)
{
        uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
        uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

        serial_puts((char *)buf, len);
}

void WaypointManager::add_local(float x, float y, float z)
{
	waypoint_t new_waypoint;
	new_waypoint.position[0] = x;
	new_waypoint.position[1] = y;
	new_waypoint.position[2] = z;

	this->waypoints.push_back(new_waypoint);
}

void WaypointManager::add_geodestic(float longitude, float latitude, float height)
{
	waypoint_t new_waypoint;
	new_waypoint.longitude = longitude;
	new_waypoint.latitude = latitude;
	new_waypoint.height = height;

	this->waypoints.push_back(new_waypoint);
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
		printf("#%d: x = %fm, y = %fm, z = %fm\n\r", i, it->position[0], it->position[1], it->position[2]);
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
			return false;
		}
	}
}

bool WaypointManager::send_mission_count_and_wait_ack()
{
	int trial = 10;
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

bool WaypointManager::send()
{
	if(this->waypoints.size() == 0) {
		return true;
	}

	/* create mavlink message reception thread */
	std::thread thread_mavlink_rx(&WaypointManager::mavlink_rx_thread_entry, this);

	send_mission_count_and_wait_ack();

	uint8_t frame = MAV_FRAME_LOCAL_NED;
	uint16_t command = 0;
	uint8_t current = 0;
	uint8_t autocontinue = 0;
	float params[4] = {0.0f};

	while(this->recvd_mission_ack == false) {
		waypoint_t waypoint;
		get_waypoint(this->mission_request_sequence, waypoint);

		int trial = 10;
        	do {
			this->recvd_mission_request_int = false;

			printf("mavlink: send waypoint #%d\n\r", this->mission_request_sequence);

			mavlink_message_t msg;
			mavlink_msg_mission_item_int_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, this->target_id, 0,
                                                               this->mission_request_sequence, 
                                                               frame, command, current, autocontinue,
                                                               params[0], params[1], params[2], params[3],
                                                               waypoint.position[0], waypoint.position[1], waypoint.position[2],
                                                               MAV_MISSION_TYPE_MISSION);
			send_mavlink_msg_to_serial(&msg);

			if(wait_mission_request_int() == true) {
				printf("next:%d\n\r", this->mission_request_sequence);
				break;
			} else {
				printf("timeout.\n\r");
			};
		} while(--trial);

		if(trial == 0) {
			this->stop_mavlink_rx_thread = true;
			thread_mavlink_rx.join();
			return false;
		}
	}

	/* stop and kill the thread */
	this->stop_mavlink_rx_thread = true;
	thread_mavlink_rx.join();

	return true;
}

void WaypointManager::mavlink_rx_thread_entry()
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

			/* pasrse the message */
			if(recvd_msg == 1) {
				switch(mavlink_recvd_msg.msgid) {
				case 51: /* MISSION_REQUEST_INT */ {
					/* decode mission request int message */
					mavlink_mission_request_int_t mission_item;
					mavlink_msg_mission_request_int_decode(&mavlink_recvd_msg, &mission_item);

					this->recvd_mission_request_int = true;
					this->mission_request_sequence = mission_item.seq;
					break;
				}
				case 47: /* MISSION_ACK */
					this->recvd_mission_ack = true;
					break;
				}
			}
		}
	}
	this->stop_mavlink_rx_thread = false;
}
