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
		cv::Mat new_img = std::move(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image);
		cv_img_queue.push(new_img);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}
}

void ROSCamDev::read(cv::Mat& ret)
{
	while(cv_img_queue.empty() == true);

	cv::Mat tmp = cv::Mat(cv_img_queue.back());
	ret = std::move(tmp);

	cv_img_queue.pop();
}

void ROSCamDev::clear()
{
	queue<cv::Mat> empty;
	std::swap(cv_img_queue, empty);
}
