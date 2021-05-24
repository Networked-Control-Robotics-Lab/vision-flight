#include <stdio.h>
#include <vector>
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

		/* one seconds timeout */
		if(elapsed_time >= 1.0f) {
			return false;
		}
	}

	return true;
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

	send_mission_count_and_wait_ack();
}
