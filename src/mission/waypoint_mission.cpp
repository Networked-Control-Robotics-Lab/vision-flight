#include <stdio.h>
#include <vector>
#include "serial.hpp"
#include "waypoint_mission.hpp"

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

bool WaypointManager::send()
{
	if(this->waypoints.size() == 0) {
		return true;
	}

	mavlink_message_t msg;
	mavlink_msg_mission_count_pack(GROUND_STATION_ID, 1, &msg, this->target_id, 0, this->waypoints.size(), MAV_MISSION_TYPE_MISSION);
	send_mavlink_msg_to_serial(&msg);
}
