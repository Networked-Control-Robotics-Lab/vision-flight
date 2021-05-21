#pragma once

#include <queue>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"

using namespace std;

class ROSCamDev
{
	queue<cv::Mat> cv_img_queue;

	ros::NodeHandle node;
	image_transport::ImageTransport img_transport;
	image_transport::Subscriber img_sub;
 
	public:
	ROSCamDev(string topic): img_transport(node)
	{
		img_sub = img_transport.subscribe(topic, 1, &ROSCamDev::cv_bridge_callback, this);
	}
 
	~ROSCamDev()
	{
	}
 
	void cv_bridge_callback(const sensor_msgs::ImageConstPtr& msg)
	{
		try {
			cv_img_queue.push(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image.clone());
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
		}
	}

	void read(cv::Mat& ret)
	{
		while(cv_img_queue.empty() == true);
		
		cv_img_queue.back().copyTo(ret);
		cv_img_queue.pop();
	}
};


void ros_thread_entry(void);
