#include <iostream>
#include <thread>
#include <ros/ros.h>
#include "apriltag_thread.hpp"
#include "shell_thread.hpp"
#include "mavlink_thread.hpp"
#include "ros_thread.hpp"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vafs");
	ros::Time::init();

	//serial_init("/dev/ttyUSB1", 115200);

	std::thread thread_ros(ros_thread_entry);
	std::thread thread_apriltag(apriltag_thread_entry);
	//std::thread thread_shell(shell_thread_entry);
	//std::thread thread_mavlink(mavlink_thread_entry);

	thread_ros.join();
	thread_apriltag.join();
	//thread_shell.join();
	//thread_mavlink.join();

	return 0;
}
