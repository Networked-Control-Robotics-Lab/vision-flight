#pragma once

#include <vector>

using namespace std;

typedef struct {
	float pos_coeff[8];
	float vel_coeff[7];
	float accel_coeff[6];
} trajectory_t;

typedef struct {
	trajectory_t x;
	trajectory_t y;
	trajectory_t z;
} trajectory3d_t;

class TrajectoryManager {
	private:
	vector<trajectory3d_t> trajs;
	int target_id;

	bool stop_mavlink_rx_thread;

	void print_trajectory(float *traj_coeff, int coeff_size);

	public:
	TrajectoryManager(int _target_id): target_id(_target_id),
                                         stop_mavlink_rx_thread(false) {}
	~TrajectoryManager() {}

	void add(trajectory_t& x, trajectory_t& y, trajectory_t& z);
	void clear();
	void get_trajectory(int index, trajectory_t& x, trajectory_t& y, trajectory_t& z);
	void print_list();
	bool send();
};
