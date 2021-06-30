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

	//copy constructor
	ROSCamDev(ROSCamDev const& rhs);

	//move constructor
	ROSCamDev(ROSCamDev&& rhs);

	//copy assignment
	ROSCamDev& operator=(ROSCamDev const& rhs);

	//move assignment
	ROSCamDev& operator=(ROSCamDev&& rhs);

	void cv_bridge_callback(const sensor_msgs::ImageConstPtr& msg);
	void read(cv::Mat& ret);
	void clear();
};

