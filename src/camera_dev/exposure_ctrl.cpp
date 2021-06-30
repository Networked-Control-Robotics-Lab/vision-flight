#include <iostream>
#include "opencv2/opencv.hpp"
#include "ros_cam.hpp"
#include "arducam_ros_ctrl.hpp"
#include "sys_time.hpp"
#include "exposure_ctrl.hpp"

using namespace std;
using namespace cv;

ExposureController::ExposureController()
{
	ros_cam_dev = new ROSCamDev("/arducam/camera/image_raw");
	this->step_size = 2000;
	this->exp_min = 0;
	this->exp_max = 3000;
	this->intensity_threshold = 0.1;
	this->exp_curr = 2500;
}

void ExposureController::convert_to_laplacian(cv::Mat& raw_img, cv::Mat& laplacian_img)
{
	int kernel_size = 3;
	int scale = 1;
	int desired_depth = CV_16S;
	int delta = 0;

	cv::Mat gaussian_img, gray_img, gradient16_img;

	cv::GaussianBlur(raw_img, gaussian_img, Size(3, 3), 0, 0, BORDER_DEFAULT);
	cv::cvtColor(gaussian_img, gray_img, COLOR_BGR2GRAY);
	cv::Laplacian(gray_img, gradient16_img, desired_depth, kernel_size, scale, delta, BORDER_DEFAULT);
	cv::convertScaleAbs(gradient16_img, laplacian_img);

#if 0
	imshow("raw image", raw_img);
	imshow("gray image", gray_img);
	imshow("laplacian image", laplacian_img);
	waitKey(1);
#endif
}

float ExposureController::calculate_average_intensity(cv::Mat& img)
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

float ExposureController::grade_laplacian(int exp, int sleep_time)
{
	cv::Mat raw_img, laplacian_img;

	arducam_ros_exposure_ctrl(exp);
	usleep(sleep_time);

	ros_cam_dev->read(raw_img);

	convert_to_laplacian(raw_img, laplacian_img);
	return calculate_average_intensity(laplacian_img);
}

void ExposureController::realtime_adjustment()
{
	cv::Mat frame;

	static float intensity_last = 0;
	static bool init = false;
	if(init == false) {
		init = true;
		ros_cam_dev->read(frame);
		intensity_last = calculate_average_intensity(frame);
		return;
	}

	/* frequency control */
	usleep(500000); //2Hz

	float intensity_now;
	ros_cam_dev->read(frame);
	intensity_now = calculate_average_intensity(frame);

	float intensity_change = intensity_now - intensity_last;
	intensity_last = intensity_now;

	if(fabs(intensity_change) > this->intensity_threshold) {
		/* gradient descent */
		float delta_exp = -intensity_change * this->step_size;
		this->exp_curr += delta_exp;

		/* bound camera exposure value */
		if(this->exp_curr > this->exp_max) {
			this->exp_curr = this->exp_max;
		} else if(this->exp_curr < this->exp_min) {
			this->exp_curr = this->exp_min;
		}

		printf("intensity change = %f, new exposure = %f\n\r",
		       intensity_change, this->exp_curr);

		arducam_ros_exposure_ctrl((int)this->exp_curr);
	}
}

int ExposureController::binary_search_adjustment(bool debug_on)
{
	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");
	int sleep_time = 150000;

	int N = 1024;
	int n = N;

	int best_interval = 0;
	int left_index, right_index;

	float grade_left, grade_right;
	float grade_max = grade_laplacian(0, sleep_time);

	int exp_left, exp_right;

	while(n > 0) {
		/* calculate next two search interval */
		left_index = best_interval - n;
		right_index = best_interval + n;

		/* calculate exposure of the intervals */
		exp_left = (int)((float)this->exp_max / (float)N * left_index);
		exp_right = (int)((float)this->exp_max / (float)N * right_index);

		/* take new pictures and calculate the grade of the exposure */
		if(left_index > 0) {
			grade_left = grade_laplacian(exp_left, sleep_time);
		}
		if(right_index < N) {
			grade_right = grade_laplacian(exp_right, sleep_time);
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
	int best_exp = (int)((float)this->exp_max / (float)N * best_interval);
	arducam_ros_exposure_ctrl(best_exp);

	if(debug_on == true) {
		printf("best exposure time = %d\n\r", best_exp);
	}

	return best_exp;
}

void ExposureController::test()
{
	ROSCamDev ros_cam_dev("/arducam/triggered/camera/image_raw");

	static int exp = 0, sign = 1;
	int max_exp = 10000;
	int delta = 1000;

	cv::Mat raw_img, gradient;

	while(1) {
		ros_cam_dev.read(raw_img);

		convert_to_laplacian(raw_img, gradient);
		calculate_average_intensity(gradient);

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
