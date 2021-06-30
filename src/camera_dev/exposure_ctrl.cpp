#include <iostream>
#include "opencv2/opencv.hpp"
#include "ros_cam.hpp"
#include "arducam_ros_ctrl.hpp"
#include "sys_time.hpp"

using namespace std;
using namespace cv;

#define OPTIMAL_EXPOSURE_BINARY_SEARCH 1

float camera_exposure_time = 2500;

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

float calculate_image_average_intensity(cv::Mat& img)
{
	int img_row = img.size().height;
	int img_col = img.size().width;
	float rescale = 1.0f / (img_row * img_col * 256);
	float intensity = 0;

	for(int r = 0; r < img_row; r++) {
		for(int c = 0; c < img_col; c++) {
			intensity += img.at<uint8_t>(r, c);
		}
	}
	intensity *= rescale;

	return intensity;
}

float grade_new_image(ROSCamDev& ros_cam_dev, int exp, int sleep_time)
{
	cv::Mat raw_img, gradient_img;

	arducam_ros_exposure_ctrl(exp);
	usleep(sleep_time);

	ros_cam_dev.clear();
	ros_cam_dev.read(raw_img);

	generate_gradient_image(raw_img, gradient_img);
	return calculate_image_gradient_strength(gradient_img);
}

int binary_search_best_camera_exposure(int max_exp, bool debug_on)
{
	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");
	int sleep_time = 150000;

	int N = 1024;
	int n = N;

	int best_interval = 0;
	int left_index, right_index;

	float grade_left, grade_right;
	float grade_max = grade_new_image(ros_cam_dev, 0, sleep_time);

	int exp_left, exp_right;

	while(n > 0) {
		/* calculate next two search interval */
		left_index = best_interval - n;
		right_index = best_interval + n;

		/* calculate exposure of the intervals */
		exp_left = (int)((float)max_exp / (float)N * left_index);
		exp_right = (int)((float)max_exp / (float)N * right_index);

		/* take new pictures and calculate the grade of the exposure */
		if(left_index > 0) {
			grade_left = grade_new_image(ros_cam_dev, exp_left, sleep_time);
		}
		if(right_index < N) {
			grade_right = grade_new_image(ros_cam_dev, exp_right, sleep_time);
		}

		/* left and right index are valid */
		if(left_index >= 0 && right_index <= N) {
			if(grade_left > grade_right > grade_max) {
				grade_max - grade_left;
				best_interval = left_index;
			} else if(grade_right > grade_left > grade_max) {
				grade_max = grade_right;
				best_interval = right_index;
			}
			/* right index is invalid */
		} else if(left_index >= 0 && right_index > N) {
			if(grade_left > grade_max) {
				grade_max = grade_left;
				best_interval = left_index;
			}
			/* left index is invalid */
		} else if(left_index < 0 && right_index <= N) {
			if(grade_right > grade_max) {
				grade_max = grade_right;
				best_interval = right_index;
			}
		}

		if(debug_on == true) {
			printf("n=%d, interval=%d, grade = %f\n\r", n, best_interval, grade_max);
		}

		/* binary serach schrinking */
		n = n / 2;
	}

	/* set camera to the best exposure value */
	int best_exp = (int)((float)max_exp / (float)N * best_interval);
	arducam_ros_exposure_ctrl(best_exp);

	if(debug_on == true) {
		printf("best exposure time = %d\n\r", best_exp);
	}

	camera_exposure_time = best_exp;

	return best_exp;
}

int full_search_best_camera_exposure(int max_exp, bool debug_on)
{
	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");

	int sleep_time = 150000; //minimum delay = 1/30s (~33333us)

	cv::Mat raw_img, gradient_img;

	float max_grad_val = 0, curr_grad_val = 0;
	int exp = 0, best_exp = 0;

	/* initial trial */
	arducam_ros_exposure_ctrl(0);
	usleep(sleep_time);
	ros_cam_dev.clear(); //make sure we are not using the old data in buffer
	ros_cam_dev.read(raw_img);
	generate_gradient_image(raw_img, gradient_img);
	max_grad_val = calculate_image_gradient_strength(gradient_img);

	/* course adjustment */
	int delta = (max_exp - 0) / 10;
	for(int i = 1; i <= 10; i++) {
		exp += delta;

		arducam_ros_exposure_ctrl(exp);
		usleep(sleep_time);

		ros_cam_dev.clear();
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

		ros_cam_dev.clear();
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

	return best_exp;
}

int scan_best_camera_exposure(int max_exp, bool debug_on)
{
#if (OPTIMAL_EXPOSURE_BINARY_SEARCH != 0)
	return binary_search_best_camera_exposure(max_exp, debug_on);
#else
	return full_search_best_camera_exposure(max_exp, debug_on);
#endif
}

void camera_exposure_gradient_descent_ctrl()
{
	static ROSCamDev ros_cam_dev("/arducam/camera/image_raw");
	static float grade_last = 0;
	static double time_last;
	static bool init = false;
	static bool fine_tune;

	cv::Mat frame;
	float step_size = 2000; //gain
	float threshold = 0.1;
	float grade_now;
	double time_now;
	float exp_min = 0, exp_max = 3000;

	if(init == false) {
		init = true;
		ros_cam_dev.clear();
		ros_cam_dev.read(frame);
		grade_last = calculate_image_average_intensity(frame);
		time_last = get_sys_time_s();
		return;
	}

	/* frequency control */
	time_now = get_sys_time_s();
	float elapsed_time = time_now - time_last;
	if(elapsed_time < 0.5) {
		return;
	} else {
		time_last = time_now;
	}

	ros_cam_dev.clear();
	ros_cam_dev.read(frame);
	grade_now = calculate_image_average_intensity(frame);

	float diff = grade_now - grade_last;

	/* phase1: detect strong light change, gradient descent of the intensity norm */
	if(fabs(diff) > threshold) {
		/* gradient descent */
		float delta_exp = -diff * step_size;
		camera_exposure_time += delta_exp;

		/* upper bound and lower bound of the exposure time */
		if(camera_exposure_time > exp_max) {
			camera_exposure_time = exp_max;
		} else if(camera_exposure_time < exp_min) {
			camera_exposure_time = exp_min;
		}

		printf("diff = %fm new exp = %f\n\r", diff, camera_exposure_time);

		arducam_ros_exposure_ctrl((int)camera_exposure_time);
	}

	/* phase2: fine tune of the exposure value to maximize the feature gradient */
}

void camera_exposure_test()
{
	ROSCamDev ros_cam_dev("/arducam/triggered/camera/image_raw");

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
