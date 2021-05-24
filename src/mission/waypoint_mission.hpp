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

	void send_mavlink_msg_to_serial(mavlink_message_t *msg);

	public:
	WaypointManager(int _target_id): target_id(_target_id) {}
	~WaypointManager() {}

	void add_local(float x, float y, float z);
	void add_geodestic(float longitude, float latitude, float height);
	void clear();
	void get_waypoint(int index, waypoint_t& waypoint);
	void print_list();
	bool send();
};
