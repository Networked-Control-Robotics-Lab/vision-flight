#pragma once

#include <ros/ros.h>
#include <arducam_usb2_ros/WriteReg.h>

void arducam_ros_exposure_ctrl(uint64_t exposure_time);
void arducam_ros_auto_mode();
void arducam_ros_trigger_mode();
