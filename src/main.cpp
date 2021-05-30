#include <iostream>
#include <thread>
#include <ros/ros.h>
#include "apriltag_thread.hpp"
#include "shell_thread.hpp"
#include "ros_thread.hpp"
#include "mission_manager.hpp"
#include "vins_mono_bridge.hpp"

using namespace std;

MissionManager mission_manager;
VINSMonoBridge *vins_mono_bridge;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vafs");
	ros::Time::init();

	/* launch vins-mono bridge */
	vins_mono_bridge = new VINSMonoBridge("/dev/ttyUSB0", 115200);
	vins_mono_bridge->launch_imu_message_listener();

	/* launch mission manager */
	//mission_manager = MissionManager(1, "/dev/ttyUSB0", 115200, WAYPOINT_CARTESIAN_FRAME, false);
	//mission_manager.launch_mavlink_listener();

	std::thread thread_ros(ros_thread_entry);
	std::thread thread_apriltag(apriltag_thread_entry);
	std::thread thread_shell(shell_thread_entry);

	thread_ros.join();
	thread_apriltag.join();
	thread_shell.join();

	delete vins_mono_bridge;

	return 0;
}
