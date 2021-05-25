#include <iostream>
#include <thread>
#include <ros/ros.h>
#include "apriltag_thread.hpp"
#include "shell_thread.hpp"
#include "ros_thread.hpp"
#include "mission_manager.hpp"

using namespace std;

MissionManager mission_manager;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vafs");
	ros::Time::init();

	mission_manager = MissionManager(1, "/dev/ttyUSB0", 115200, WAYPOINT_CARTESIAN_FRAME, false);
	mission_manager.launch_mavlink_listener();

	std::thread thread_ros(ros_thread_entry);
	std::thread thread_apriltag(apriltag_thread_entry);
	std::thread thread_shell(shell_thread_entry);

	thread_ros.join();
	thread_apriltag.join();
	thread_shell.join();

	return 0;
}
