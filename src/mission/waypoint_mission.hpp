#pragma once

#include <vector>

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
}

enum {
	WAYPOINT_CARTESIAN_FRAME,
	WAYPOINT_GEODETIC_FRAME	
} WaypointManagerFrame;

using namespace std;

typedef struct {
	float x;
	float y;
	float z;
} waypoint_t;

class WaypointManager {
	private:
	vector<waypoint_t> waypoints;
	int target_id;
	int frame;

	/* mission request int message */
	bool recvd_mission_request_int;
	int mission_request_sequence;

	/* mission ack message */
	bool recvd_mission_ack;

	/* command ack message */
	bool recvd_cmd_long_ack;

	int serial_fd;

	void serial_puts(char *s, size_t size);
	void send_mavlink_msg_to_serial(mavlink_message_t *msg);
	bool wait_mission_request_int();
	bool wait_mission_ack();
	bool send_mission_count_and_wait_ack();
	bool send_mission_waypoint(int index, bool is_last_waypoint);
	bool wait_command_long_ack();


	public:
	WaypointManager() {}
	WaypointManager(int _target_id, int _frame, int _serial_fd): target_id(_target_id),
                                                                     frame(_frame),
                                                                     recvd_mission_request_int(false),
                                                                     recvd_mission_ack(false),
                                                                     serial_fd(_serial_fd) {}
	~WaypointManager() {}

	//copy constructor
	WaypointManager(WaypointManager const& rhs): waypoints(rhs.waypoints),
                                                     target_id(rhs.target_id),
                                                     frame(rhs.frame),
                                                     recvd_mission_request_int(rhs.recvd_mission_request_int),
                                                     mission_request_sequence(rhs.mission_request_sequence),
                                                     recvd_mission_ack(rhs.recvd_mission_ack),
                                                     serial_fd(rhs.serial_fd) {}

	//move constructor
	WaypointManager(WaypointManager&& rhs): waypoints(std::move(rhs.waypoints)),
                                                target_id(std::move(rhs.target_id)),
                                                frame(std::move(rhs.frame)),
                                                recvd_mission_request_int(std::move(rhs.recvd_mission_request_int)),
                                                mission_request_sequence(std::move(rhs.mission_request_sequence)),
                                                recvd_mission_ack(std::move(rhs.recvd_mission_ack)),
                                                serial_fd(std::move(rhs.serial_fd)) {}

	//copy assignment
	WaypointManager& operator=(WaypointManager const& rhs)
	{
		if(this != &rhs) {
			waypoints = rhs.waypoints;
			target_id = rhs.target_id;
			frame = rhs.frame;
			recvd_mission_request_int = rhs.recvd_mission_request_int;
			mission_request_sequence = rhs.mission_request_sequence;
			recvd_mission_ack = rhs.recvd_mission_ack;
			serial_fd = rhs.serial_fd;
		}
		return *this;
	}

	//move assignment
	WaypointManager& operator=(WaypointManager&& rhs)
	{
		if(this != &rhs) {
			std::swap(waypoints, rhs.waypoints);
			std::swap(target_id, rhs.target_id);
			std::swap(frame, rhs.frame);
			std::swap(recvd_mission_request_int, rhs.recvd_mission_request_int);
			std::swap(mission_request_sequence, rhs.mission_request_sequence);
			std::swap(recvd_mission_ack, rhs.recvd_mission_ack);
			std::swap(serial_fd, rhs.serial_fd);
		}
		return *this;
	}

	void mavlink_rx_message_handler(mavlink_message_t& msg);

	void add(float x, float y, float z);
	int  size();
	void clear();
	void get_waypoint(int index, waypoint_t& waypoint);
	void print_list();
	bool send_mission();
	void send_start_cmd();
	bool send_takeoff_cmd();
	bool send_land_cmd();
	void send_halt_cmd();
	void send_resume_cmd();
	void send_goto_cmd(float yaw, float x, float y, float z);
};
