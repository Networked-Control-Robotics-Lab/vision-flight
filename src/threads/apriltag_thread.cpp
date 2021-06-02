#include <iostream>
#include "opencv2/opencv.hpp"
#include "apriltag_utils.hpp"
#include "ros_cam.hpp"
#include "arducam_ros_ctrl.hpp"
#include "sys_time.hpp"
#include "exposure_ctrl.hpp"

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

using namespace std;
using namespace cv;

cv::Mat adjust_contrast(cv::Mat& raw_img, double alpha, double beta)
{
	cv::Mat contrast_img;
	raw_img.convertTo(contrast_img, -1, alpha, beta);

	return std::move(contrast_img);
}

void send_ros_debug_image(ros::Publisher& debug_img_publisher, cv::Mat img)
{
	putText(img, "Navigation invalid", Point(5, 20), FONT_HERSHEY_COMPLEX_SMALL, 1.0, Scalar(0, 0, 255));

	sensor_msgs::ImagePtr debug_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
	debug_img_publisher.publish(debug_img_msg);
}

void apriltag_thread_entry(void)
{
	ROSCamDev ros_cam_dev("/arducam/triggered/camera/image_raw");

	ros::NodeHandle node("vision_flight");
	ros::Publisher debug_img_publisher = node.advertise<sensor_msgs::Image>("debug_img", 20);

	/* camera initialization */
	//VideoCapture camera(0);
	//camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_IMAGE_WIDTH);
	//camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_IMAGE_HEIGHT);
	//if (!camera.isOpened()) {
	//	cerr << "couldn't open video camerature device" << endl;
	//	exit(-1);
	//}

	/* camera parameters */
	float fx = 411.64591;
	float fy = 411.72171;
	float cx = 306.34861;
	float cy = 220.31898;

	/* infomation of tag to be detected */
	float tag_size = 0.16; //[m]
	float tag_id = 0;

	/* apriltag detector setup */
	apriltag_family_t *tf = NULL;
	tf = tag36h11_create();

	apriltag_detector_t *td = apriltag_detector_create();
	apriltag_detector_add_family(td, tf);
	td->quad_decimate = 2.0f;
	td->quad_sigma = 0.0f;
	td->nthreads = 1;
	td->debug = 0;
	td->refine_edges = 1;

	apriltag_detection_info_t tag_info;
	tag_info.tagsize = tag_size;
	tag_info.fx = fx;
	tag_info.fy = fy;
	tag_info.cx = cx;
	tag_info.cy = cy;

	float last_time = get_sys_time_s();
	float curr_time;
	float elapsed_time;
	float apriltag_update_rate = 5;

	Mat raw_img, gray, gradient, contrast_img;
	while (true) {
		ros_cam_dev.clear();
		ros_cam_dev.read(raw_img);

		curr_time = get_sys_time_s();
		elapsed_time = curr_time - last_time;
		if(elapsed_time >= (1 / apriltag_update_rate)) {
			last_time = get_sys_time_s();
			send_ros_debug_image(debug_img_publisher, raw_img);
		} else {
			continue;
		}

		contrast_img = adjust_contrast(raw_img,  2, 0);

		//camera >> raw_img;
		cvtColor(contrast_img, gray, COLOR_BGR2GRAY);

		/* convert image data to apriltag's format */
		image_u8_t im = {
			.width = gray.cols,
			.height = gray.rows,
			.stride = gray.cols,
			.buf = gray.data
		};

		/* tags detection */
		zarray_t *detections = apriltag_detector_detect(td, &im);

		/* localization */
		apriltag_pose_t pose;
		if(tag_localize(tag_id, detections, tag_info, pose) == true) {
			cout << "t:\n"
			     << pose.t->data[0] << "  " <<pose.t->data[1] << " " << pose.t->data[2] << endl;
			cout << "R:\n"
			     << pose.R->data[0] << ", " << pose.R->data[1] << ", " << pose.R->data[2] << endl
			     << pose.R->data[3] << ", " << pose.R->data[4] << ", " << pose.R->data[5] << endl
			     << pose.R->data[6] << ", " << pose.R->data[7] << ", " << pose.R->data[8] << endl;
			cout << "--------------------------------" << endl;
		}

		/* visualization */
		tags_visualize(raw_img, detections);

		send_ros_debug_image(debug_img_publisher, raw_img);

		apriltag_detections_destroy(detections);
	}

	apriltag_detector_destroy(td);
	tag36h11_destroy(tf);
}
