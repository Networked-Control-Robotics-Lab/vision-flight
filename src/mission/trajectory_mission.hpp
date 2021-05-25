#pragma once

#include <stdint.h>
#include <vector>
#include "serial.hpp"

extern "C" {
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "../lib/mavlink_v2/ncrl_mavlink/ncrl_mavlink.h"
}

using namespace std;

enum {
	TRAJECTORY_TYPE_X,
	TRAJECTORY_TYPE_Y,
	TRAJECTORY_TYPE_Z
} TrajectoryType;

typedef struct {
	float coeff[8];
} trajectory_t;

typedef struct {
	trajectory_t x;
	trajectory_t y;
	trajectory_t z;
	float flight_time;
} trajectory3d_t;

class TrajectoryManager {
	private:
	vector<trajectory3d_t> trajs;
	int target_id;

	bool stop_mavlink_rx_thread;

	bool z_enabled;
	bool yaw_enabled;

	/* mavlink variables */
	bool recvd_traj_ack;
	uint8_t traj_ack_val;

	void send_mavlink_msg_to_serial(mavlink_message_t *msg);
	void print_trajectory(float *coeff, int coeff_size);
	bool wait_trajectory_ack();
	bool send_traj_write_and_wait_ack();
	bool send_traj_item_and_wait_ack(uint8_t index, uint8_t type);
	void mavlink_rx_message_handler(mavlink_message_t& msg);
	void mavlink_rx_thread_entry();

	public:
	TrajectoryManager(int _target_id, bool _z_enabled, bool _yaw_enabled): target_id(_target_id),
                                                                               z_enabled(_z_enabled),
                                                                               yaw_enabled(_yaw_enabled),
                                                                               recvd_traj_ack(false),
                                                                               stop_mavlink_rx_thread(false) {}
	~TrajectoryManager() {}

	//copy constructor
	TrajectoryManager(TrajectoryManager const& rhs);

	//move constructor
	TrajectoryManager(TrajectoryManager&& rhs);

	//copy assignment
	TrajectoryManager& operator=(TrajectoryManager const& rhs);

	//move assignment
	TrajectoryManager& operator=(TrajectoryManager&& rhs);

	void add(trajectory_t& x, trajectory_t& y, trajectory_t& z, float flight_time);
	void clear();
	void get_trajectory(int index, trajectory_t& x, trajectory_t& y, trajectory_t& z);
	void print_list();
	bool send_mission();
	void send_takeoff_cmd();
	void send_land_cmd();
	void start(bool looping);
	void stop();
};
