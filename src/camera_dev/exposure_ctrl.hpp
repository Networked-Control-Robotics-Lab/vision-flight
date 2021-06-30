#pragma once

#include "ros_cam.hpp"

class ExposureController
{
private:
	ROSCamDev* ros_cam_dev;
	float exp_min;
	float exp_max;
	float exp_curr;

	float step_size;
	float intensity_threshold;
	float intensity_last;

	void convert_to_laplacian(cv::Mat& raw_img, cv::Mat& laplacian_img);
	float calculate_average_intensity(cv::Mat& img);
	float grade_laplacian(int exp, int sleep_time);
	void test();

public:
	ExposureController();
	~ExposureController()
	{
		delete ros_cam_dev;
	}

	void realtime_adjustment(bool debug_on);
	int binary_search_adjustment(bool debug_on);

	//copy constructor
	ExposureController(ExposureController const& rhs) = delete;

	//move constructor
	ExposureController(ExposureController&& rhs) = delete;

	//copy assignment
	ExposureController& operator=(ExposureController const& rhs) = delete;

	//move assignment
	ExposureController& operator=(ExposureController&& rhs) = delete;
};


void camera_exposure_test();
