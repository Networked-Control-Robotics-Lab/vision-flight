#include <ros/ros.h>

double get_sys_time_s()
{
	return ros::Time::now().toSec();
}
