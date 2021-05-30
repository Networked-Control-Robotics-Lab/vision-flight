#include <iostream>
#include <string>
#include <queue>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include "opencv2/opencv.hpp"
#include "vins_mono_bridge.hpp"

using namespace std;
using namespace cv;

extern VINSMonoBridge *vins_mono_bridge;

void vins_mono_callback(nav_msgs::Odometry vio_ros_msg)
{
	vins_mono_bridge->send_pose_to_serial(vio_ros_msg);
}

void ros_thread_entry(void)
{
	ros::NodeHandle node;
	ros::Subscriber vins_mono_subscriber = node.subscribe("vins_estimator/imu_propagate", 1000, vins_mono_callback);

	ros::spin();
}
