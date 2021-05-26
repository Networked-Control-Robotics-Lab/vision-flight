#include <iostream>
#include "opencv2/opencv.hpp"
#include "apriltag_utils.hpp"
#include "ros_cam.hpp"
#include "arducam_ros_ctrl.hpp"
#include "sys_time.hpp"

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

void generate_gradient_image(cv::Mat& frame, cv::Mat& gradient)
{
        int kernel_size = 3;
        int scale = 1;
        int ddepth = CV_16S;
        int delta = 0;

	cv::Mat gray, gradient16;

	GaussianBlur(frame, frame, Size(3, 3), 0, 0, BORDER_DEFAULT );
	cvtColor(frame, gray, COLOR_BGR2GRAY);
	Laplacian(gray, gradient16, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(gradient16, gradient);

	imshow("gray", gray);
	imshow("gradient", gradient);
	waitKey(1);
}

float calculate_image_gradient_strength(cv::Mat& image)
{
	float rescale = 1.0f / (image.size().width * image.size().height);
	float gradient_strength = 0;

	for(int r = 0; r < image.size().width; r++) {
		for(int c = 0; c < image.size().height; c++) {
			gradient_strength += image.at<uint8_t>(r, c, 0) / 256.0f;
		}
	}
	gradient_strength *= rescale;

	return gradient_strength;
}

void scan_best_camera_exposure(ROSCamDev& ros_cam_dev, int exposure_max, int exposure_increment)
{
	cv::Mat frame, gradient;

	float gradient_strength_max = 0, curr_gradient = 0;
	int best_exposure = 0;

	arducam_ros_exposure_ctrl(0);
	ros_cam_dev.read(frame);	

	generate_gradient_image(frame, gradient);
	gradient_strength_max = calculate_image_gradient_strength(gradient);

	for(int exposure = exposure_increment; exposure <= exposure_max; exposure += exposure_increment) {
		arducam_ros_exposure_ctrl(exposure);

		cv::Mat frame, gradient;
		ros_cam_dev.read(frame);

		sleep(1);

		generate_gradient_image(frame, gradient);
		curr_gradient = calculate_image_gradient_strength(gradient);
		printf("curr gradient = %f\n\r", curr_gradient);

		if(curr_gradient > gradient_strength_max) {
			best_exposure = exposure;
			gradient_strength_max = curr_gradient;
		}
	}

	arducam_ros_exposure_ctrl(best_exposure);
	printf("best exposure: %f\n\r", best_exposure);
}

void camera_exposure_test(ROSCamDev& ros_cam_dev)
{
	static int exp = 0, sign = 1;

	cv::Mat frame, gradient;

	generate_gradient_image(frame, gradient);
	calculate_image_gradient_strength(gradient);

	exp += sign * 1000;

	if(exp <= 0) {
		exp = 0;
		sign *= -1;
	}

	if(exp >= 10000) {
		exp = 10000;
		sign *= -1;
	}

	arducam_ros_exposure_ctrl(exp);
}

void apriltag_thread_entry(void)
{
	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");

	scan_best_camera_exposure(ros_cam_dev, 10000, 1000);


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

	int kernel_size = 3;
	int scale = 1;
	int ddepth = CV_16S;
	int delta = 0;
	Mat frame, gray, gradient;
	while (true) {
		ros_cam_dev.read(frame);

//		generate_gradient_image(frame, gradient);
//		calculate_image_gradient_strength(gradient);
//		camera_exposure_ctrl();

//		continue;

		//camera >> frame;
		cvtColor(frame, gray, COLOR_BGR2GRAY);

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
		tags_visualize(frame, detections);
		imshow("Tag Detections", frame);
		if (waitKey(30) >= 0) {
			break;
		}

		apriltag_detections_destroy(detections);
	}

	apriltag_detector_destroy(td);
	tag36h11_destroy(tf);
}
