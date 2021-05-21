#include <iostream>
#include <string>
#include <queue>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

void ros_thread_entry(void)
{
	ros::spin();
}
