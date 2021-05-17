#pragma once

#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

void tags_visualize(cv::Mat& frame, zarray_t* detections);
bool tag_localize(int id, zarray_t* detections, apriltag_detection_info_t& tag_info, apriltag_pose_t& pose);
