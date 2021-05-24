#pragma

#include <vector>

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
}

using namespace std;

typedef struct {
	/* local ned coordinate system */
	float position[3];

	/* geodestic coordicate system */
	float longitude;
	float latitude;
	float height;
} waypoint_t;

class WaypointManager {
	private:
	vector<waypoint_t> waypoints;
	int target_id;

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
	bool send_mission_waypoint(int index);
	void mavlink_rx_thread_entry(void);

	public:
	WaypointManager(int _target_id): target_id(_target_id),
                                         stop_mavlink_rx_thread(false),
                                         recvd_mission_request_int(false),
                                         recvd_mission_ack(false) {}
	~WaypointManager() {}

	void add_local(float x, float y, float z);
	void add_geodestic(float longitude, float latitude, float height);
	void clear();
	void get_waypoint(int index, waypoint_t& waypoint);
	void print_list();
	bool send();
};
