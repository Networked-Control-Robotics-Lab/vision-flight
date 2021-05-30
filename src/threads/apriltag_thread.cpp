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

void apriltag_thread_entry(void)
{
	scan_best_camera_exposure(5000, false);

	ROSCamDev ros_cam_dev("/arducam/triggered/camera/image_raw");

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

	float start_time = get_sys_time_s();
	float curr_time;

	Mat raw_img, gray, gradient, contrast_img;
	while (true) {
		ros_cam_dev.read(raw_img);

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

		/* visualization with opencv */
		tags_visualize(raw_img, detections);
		imshow("Tag Detections", raw_img);
		if (waitKey(30) >= 0) {
			break;
		}

		apriltag_detections_destroy(detections);
	}

	apriltag_detector_destroy(td);
	tag36h11_destroy(tf);
}
