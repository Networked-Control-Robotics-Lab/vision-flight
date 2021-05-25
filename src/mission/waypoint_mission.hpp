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

	bool stop_mavlink_rx_thread;

	/* mission request int message */
	bool recvd_mission_request_int;
	int mission_request_sequence;

	/* mission ack message */
	bool recvd_mission_ack;

	void send_mavlink_msg_to_serial(mavlink_message_t *msg);
	bool wait_mission_request_int();
	bool wait_mission_ack();
	bool send_mission_count_and_wait_ack();
	bool send_mission_waypoint(int index, bool is_last_waypoint);
	void mavlink_rx_message_handler(mavlink_message_t& msg);
	void mavlink_rx_thread_entry(void);

	public:
	WaypointManager() {}
	WaypointManager(int _target_id, int _frame): target_id(_target_id),
                                                     frame(_frame),
                                                     stop_mavlink_rx_thread(false),
                                                     recvd_mission_request_int(false),
                                                     recvd_mission_ack(false) {}
	~WaypointManager() {}

	//copy constructor
	WaypointManager(WaypointManager const& rhs): waypoints(rhs.waypoints),
                                                     target_id(rhs.target_id),
                                                     frame(rhs.frame),
                                                     stop_mavlink_rx_thread(rhs.stop_mavlink_rx_thread),
                                                     recvd_mission_request_int(rhs.recvd_mission_request_int),
                                                     mission_request_sequence(rhs.mission_request_sequence),
                                                     recvd_mission_ack(rhs.recvd_mission_ack) {}

	//move constructor
	WaypointManager(WaypointManager&& rhs): waypoints(std::move(rhs.waypoints)),
                                                target_id(std::move(rhs.target_id)),
                                                frame(std::move(rhs.frame)),
                                                stop_mavlink_rx_thread(std::move(rhs.stop_mavlink_rx_thread)),
                                                recvd_mission_request_int(std::move(rhs.recvd_mission_request_int)),
                                                mission_request_sequence(std::move(rhs.mission_request_sequence)),
                                                recvd_mission_ack(std::move(rhs.recvd_mission_ack)) {}

	//copy assignment
	WaypointManager& operator=(WaypointManager const& rhs) {
		if(this == &rhs) {
			return *this;
		}

		waypoints = rhs.waypoints;
		target_id = rhs.target_id;
		frame = rhs.frame;
		stop_mavlink_rx_thread = rhs.stop_mavlink_rx_thread;
		recvd_mission_request_int = rhs.recvd_mission_request_int;
		mission_request_sequence = rhs.mission_request_sequence;
		recvd_mission_ack = rhs.recvd_mission_ack;
	}

	//move assignment
	WaypointManager& operator=(WaypointManager&& rhs) {
		if(this == &rhs) {
			return *this;
		}

		waypoints = std::move(rhs.waypoints);
		target_id = std::move(rhs.target_id);
		frame = std::move(rhs.frame);
		stop_mavlink_rx_thread = std::move(rhs.stop_mavlink_rx_thread);
		recvd_mission_request_int = std::move(rhs.recvd_mission_request_int);
		mission_request_sequence = std::move(rhs.mission_request_sequence);
		recvd_mission_ack = std::move(rhs.recvd_mission_ack);
	}

	void add(float x, float y, float z);
	void clear();
	void get_waypoint(int index, waypoint_t& waypoint);
	void print_list();
	bool send_mission();
	void send_start_cmd();
	void send_takeoff_cmd();
	void send_land_cmd();
	void send_halt_cmd();
	void send_resume_cmd();
};
