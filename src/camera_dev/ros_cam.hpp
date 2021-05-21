#pragma once

#include <queue>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"

using namespace std;

class ROSCamDev
{
	private:
	queue<cv::Mat> cv_img_queue;

	ros::NodeHandle node;
	image_transport::ImageTransport img_transport;
	image_transport::Subscriber img_sub;
 
	public:
	ROSCamDev(string topic);
	~ROSCamDev() {}
 
	void cv_bridge_callback(const sensor_msgs::ImageConstPtr& msg);
	void read(cv::Mat& ret);
};

