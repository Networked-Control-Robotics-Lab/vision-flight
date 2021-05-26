#pragma once

#include <string>
#include "waypoint_mission.hpp"
#include "trajectory_mission.hpp"

#define RETRY_TIME_MAX 5

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
		if(this->thread_mavlink_rx == nullptr) {
			this->thread_mavlink_rx = new std::thread(&MissionManager::mavlink_rx_thread_entry, std::ref(*this));
		}
	}

	~MissionManager()
	{
		if(this->thread_mavlink_rx != nullptr) {
			thread_mavlink_rx->join();
			delete thread_mavlink_rx;
		}
	}

	//copy constructor
	MissionManager(MissionManager const& rhs)
	{
		/* error of copy constructing with thread running */
		if(rhs.thread_mavlink_rx != nullptr) {
			printf("[MissionManager] error: you are copy constructing a object which contains a running thread.\n\r");
			exit(-1);
		}

		serial_fd = rhs.serial_fd;
		thread_mavlink_rx = rhs.thread_mavlink_rx;
		waypoint = rhs.waypoint;
		trajectory = rhs.trajectory;
	}

	//move constructor
	MissionManager(MissionManager&& rhs)
	{
		/* error of move constructing with thread running */
		if(rhs.thread_mavlink_rx != nullptr) {
			printf("[MissionManager] error: you are move constructing a object which contains a running thread.\n\r");
			exit(-1);
		}

		std::swap(serial_fd, rhs.serial_fd);
		std::swap(thread_mavlink_rx, rhs.thread_mavlink_rx);
		std::swap(waypoint, rhs.waypoint);
		std::swap(trajectory, rhs.trajectory);
	}

	//copy assignment
	MissionManager& operator=(MissionManager const& rhs)
	{
		if(this != &rhs) {
			/* error of move constructing with thread running */
			if(rhs.thread_mavlink_rx != nullptr) {
				printf("[MissionManager] error: you are copy assigning a object which contains a running thread.\n\r");
				exit(-1);
			}

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
			/* error of move constructing with thread running */
			if(rhs.thread_mavlink_rx != nullptr) {
				printf("[MissionManager] error: you are move assigning a object which contains a running thread.\n\r");
				exit(-1);
			}

			std::swap(serial_fd, rhs.serial_fd);
			std::swap(thread_mavlink_rx, rhs.thread_mavlink_rx);
			std::swap(waypoint, rhs.waypoint);
			std::swap(trajectory, rhs.trajectory);
		}
		return *this;
	}
};
