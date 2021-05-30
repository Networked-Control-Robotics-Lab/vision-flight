#include <iostream>
#include "opencv2/opencv.hpp"
#include "ros_cam.hpp"
#include "arducam_ros_ctrl.hpp"
#include "sys_time.hpp"

using namespace std;
using namespace cv;

void generate_gradient_image(cv::Mat& raw_img, cv::Mat& gradient_img)
{
        int kernel_size = 3;
        int scale = 1;
        int desired_depth = CV_16S;
        int delta = 0;

	cv::Mat gaussian_img, gray_img, gradient16_img;

	cv::GaussianBlur(raw_img, gaussian_img, Size(3, 3), 0, 0, BORDER_DEFAULT);
	cv::cvtColor(gaussian_img, gray_img, COLOR_BGR2GRAY);
	cv::Laplacian(gray_img, gradient16_img, desired_depth, kernel_size, scale, delta, BORDER_DEFAULT);
	cv::convertScaleAbs(gradient16_img, gradient_img);

#if 0
	imshow("raw image", raw_img);
	imshow("gray image", gray_img);
	imshow("laplacian image", gradient_img);
	waitKey(1);
#endif
}

float calculate_image_gradient_strength(cv::Mat& gradient_img)
{
	int img_row = gradient_img.size().height;
	int img_col = gradient_img.size().width;

	float rescale = 1.0f / (img_row * img_col);
	float gradient_strength = 0;

	for(int r = 0; r < img_row; r++) {
		for(int c = 0; c < img_col; c++) {
			gradient_strength += gradient_img.at<uint8_t>(r, c) / 256.0f;
		}
	}
	gradient_strength *= rescale;

	return gradient_strength;
}

void scan_best_camera_exposure(int max_exp, bool debug_on)
{
	ROSCamDev ros_cam_dev("/cam0/arducam/triggered/camera/image_raw");

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
	ROSCamDev ros_cam_dev("/cam0/arducam/triggered/camera/image_raw");

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
