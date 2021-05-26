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

void generate_gradient_image(cv::Mat& raw_img, cv::Mat& gradient_img)
{
        int kernel_size = 3;
        int scale = 1;
        int desired_depth = CV_16S;
        int delta = 0;

	cv::Mat gray_img, gradient16_img;

	cv::GaussianBlur(raw_img, raw_img, Size(3, 3), 0, 0, BORDER_DEFAULT );
	cv::cvtColor(raw_img, gray_img, COLOR_BGR2GRAY);
	cv::Laplacian(gray_img, gradient16_img, desired_depth, kernel_size, scale, delta, BORDER_DEFAULT);
	cv::convertScaleAbs(gradient16_img, gradient_img);

	imshow("raw image", raw_img);
	imshow("gray image", gray_img);
	imshow("laplacian image", gradient_img);
	waitKey(1);
}

float calculate_image_gradient_strength(cv::Mat& gradient_img)
{
	int img_row = gradient_img.size().width;
	int img_col = gradient_img.size().height;

	float rescale = 1.0f / (img_row * img_col);
	float gradient_strength = 0;

	for(int r = 0; r < img_row; r++) {
		for(int c = 0; c < img_col; c++) {
			gradient_strength += gradient_img.at<uint8_t>(r, c, 0) / 256.0f;
		}
	}
	gradient_strength *= rescale;

	return gradient_strength;
}

void scan_best_camera_exposure(ROSCamDev& ros_cam_dev, int max_exp, bool debug_on)
{
	int sleep_time = 150000; //minimum delay = 1/30s (~33333us)

	cv::Mat raw_img, gradient_img;

	float max_grad_val = 0, curr_grad_val = 0;
	int exp = 0, best_exp = 0;

	/* initial trial */
	arducam_ros_exposure_ctrl(0);
	ros_cam_dev.read(raw_img);	
	usleep(sleep_time);
	generate_gradient_image(raw_img, gradient_img);
	max_grad_val = calculate_image_gradient_strength(gradient_img);

	/* course adjustment */
	int delta = (max_exp - 0) / 10;
	for(int i = 1; i <= 10; i++) {
		exp += delta;

		arducam_ros_exposure_ctrl(exp);
		usleep(sleep_time);

		ros_cam_dev.read(raw_img);

		generate_gradient_image(raw_img, gradient_img);
		curr_grad_val = calculate_image_gradient_strength(gradient_img);

		if(debug_on) {
			printf("exposure = %d, gradient value = %f\n\r", exp, curr_grad_val);
		}

		if(curr_grad_val > max_grad_val) {
			best_exp = exp;
			max_grad_val = curr_grad_val;
		}
	}

	arducam_ros_exposure_ctrl(best_exp);
	if(debug_on) {
		printf("course tuned best exposure value =  %d\n\r", best_exp);
	}

	/* fine adjustment */
	int search_range = delta * 3;
	int search_start, search_end;

	if(best_exp + (search_range / 2) > max_exp) {
		search_start = max_exp - search_range;
		search_end = max_exp;
	} else if(best_exp - (search_range / 2) < 0) {
		search_start = 0;
		search_end = search_range;
	} else {
		search_start = best_exp - (search_range / 2);
		search_end = best_exp + (search_range / 2);
	}

	delta = search_range / 50;
	for(exp = search_start; exp < search_end; exp += delta) {
		arducam_ros_exposure_ctrl(exp);
		usleep(sleep_time);

		ros_cam_dev.read(raw_img);

		generate_gradient_image(raw_img, gradient_img);
		curr_grad_val = calculate_image_gradient_strength(gradient_img);

		if(debug_on) {
			printf("exposure = %d, gradient value = %f\n\r", exp, curr_grad_val);
		}

		if(curr_grad_val > max_grad_val) {
			best_exp = exp;
			max_grad_val = curr_grad_val;
		}
	}

	arducam_ros_exposure_ctrl(best_exp);
	if(debug_on) {
		printf("fine tuned best exposure value =  %d\n\r", best_exp);
	}
}

void camera_exposure_test()
{
	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");

	static int exp = 0, sign = 1;
	int max_exp = 10000;
	int delta = 1000;

	cv::Mat raw_img, gradient;

	while(1) {
		ros_cam_dev.read(raw_img);

		generate_gradient_image(raw_img, gradient);
		calculate_image_gradient_strength(gradient);

		exp += sign * delta;

		if(exp <= 0) {
			exp = 0;
			sign *= -1;
		}

		if(exp >= max_exp) {
			exp = max_exp;
			sign *= -1;
		}

		arducam_ros_exposure_ctrl(exp);
	}
}

void apriltag_thread_entry(void)
{
	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");

	scan_best_camera_exposure(ros_cam_dev, 10000, true);


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

	Mat raw_img, gray, gradient;
	while (true) {
		ros_cam_dev.read(raw_img);

		//camera >> raw_img;
		cvtColor(raw_img, gray, COLOR_BGR2GRAY);

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
