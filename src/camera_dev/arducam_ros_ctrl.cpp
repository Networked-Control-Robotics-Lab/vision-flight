#include <ros/ros.h>
#include <arducam_usb2_ros/WriteReg.h>

void arducam_ros_exposure_ctrl(uint64_t exposure_time)
{
	arducam_usb2_ros::WriteReg write_reg_srv;
	write_reg_srv.request.reg = 0x3012;
	write_reg_srv.request.val = exposure_time;
	ros::service::call("/arducam/write_reg", write_reg_srv);
}

