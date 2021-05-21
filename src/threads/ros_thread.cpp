#include <iostream>
#include <string>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class ROSCamDev
{
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
		cv_bridge::CvImagePtr cv_img_ptr;
		try {
			cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
		}
 
		cv::imshow("test", cv_img_ptr->image);
		cv::waitKey(3);
	}
};

void ros_thread_entry(void)
{
	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");
	ros::spin();
}
