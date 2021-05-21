#include <queue>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"
#include "ros_cam.hpp"

using namespace std;

ROSCamDev::ROSCamDev(string topic): img_transport(node)
{
	img_sub = img_transport.subscribe(topic, 1, &ROSCamDev::cv_bridge_callback, this);
}

void ROSCamDev::cv_bridge_callback(const sensor_msgs::ImageConstPtr& msg)
{
	try {
		cv_img_queue.push(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone());
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}
}

void ROSCamDev::read(cv::Mat& ret)
{
	while(cv_img_queue.empty() == true);
	
	cv_img_queue.back().copyTo(ret);
	cv_img_queue.pop();
}
