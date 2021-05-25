#pragma once

#include <string>
#include "waypoint_mission.hpp"
#include "trajectory_mission.hpp"

class MissionManager {
	private:
	int serial_fd;
	std::thread *thread_mavlink_rx;

	void open_serial_port(string& port_name, int baudrate);
	int serial_getc(char *c);
	void mavlink_rx_thread_entry();

	public:
	WaypointManager waypoint;
	TrajectoryManager trajectory;

	MissionManager(): thread_mavlink_rx(nullptr) {};
	MissionManager(int target_id, string serial_port, int baudrate,
                       int frame, bool traj_z_enabled);

	void launch_mavlink_listener()
	{
		this->thread_mavlink_rx = new std::thread(&MissionManager::mavlink_rx_thread_entry, std::ref(*this));
	}

	~MissionManager()
	{
		if(this->thread_mavlink_rx != nullptr) {
			thread_mavlink_rx->join();
			delete thread_mavlink_rx;
		}
	}

	//copy constructor
	MissionManager(MissionManager const& rhs): serial_fd(rhs.serial_fd),
                                                   thread_mavlink_rx(rhs.thread_mavlink_rx),
                                                   waypoint(rhs.waypoint),
                                                   trajectory(rhs.trajectory) {}

	//move constructor
	MissionManager(MissionManager&& rhs): serial_fd(std::move(rhs.serial_fd)),
                                              thread_mavlink_rx(std::move(rhs.thread_mavlink_rx)),
                                              waypoint(std::move(rhs.waypoint)),
                                              trajectory(std::move(rhs.trajectory)) {}

	//copy assignment
	MissionManager& operator=(MissionManager const& rhs)
	{
		if(this != &rhs) {
			serial_fd = rhs.serial_fd;
			thread_mavlink_rx = rhs.thread_mavlink_rx;
			waypoint = rhs.waypoint;
			trajectory = rhs.trajectory;
		}
		return *this;
	}

	//move assignment
	MissionManager& operator=(MissionManager&& rhs)
	{
		if(this != &rhs) {
			std::swap(serial_fd, rhs.serial_fd);
			std::swap(thread_mavlink_rx, rhs.thread_mavlink_rx);
			std::swap(waypoint, rhs.waypoint);
			std::swap(trajectory, rhs.trajectory);
		}
		return *this;
	}
};
