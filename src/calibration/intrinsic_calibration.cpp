#include <iostream>
#include <thread>
#include <iomanip>
#include <opencv2/opencv.hpp>
#include "ros_cam.hpp"

#define SQUARE_SIZE  0.29 //[cm]
#define BOARD_WIDTH  6
#define BOARD_HEIGHT 8

#define IMG_SAMPLE_SIZE 15

using namespace std;
using namespace cv;

//input image
cv::Mat raw_image;
int image_width;
int image_height;

//calibration image names
vector<string> image_names;

//checkerboard parameters
Size board_size = Size(BOARD_WIDTH, BOARD_HEIGHT);
Size image_size;
vector<vector<Point2f>> image_2d_points;
vector<vector<Point3f>> object_3d_points;
int image_count = 1;
bool checkerboard_detected = false;

//intrinsic parameters
Mat camera_matrix, dist_coeffs;
vector<Mat> rvecs, tvecs;

void set_img_filenames(void)
{
	image_names.clear();

	char img_path[100] = "";

	for(int i = 1; i <= IMG_SAMPLE_SIZE; i++) {
		sprintf(img_path, "/tmp/intrinsic_%dx%d_%d.jpg", image_width, image_height, i);
		image_names.push_back(img_path);
	}
}

void add_board_points(vector<Point2f> &_2d_corners, vector<Point3f> &_3d_corners)
{
	image_2d_points.push_back(_2d_corners);
	object_3d_points.push_back(_3d_corners);
}

bool visualize_checkerboard(Mat &raw_image, Mat &board_visualized_img)
{
	Mat gray_image;
	cv::cvtColor(raw_image, gray_image, cv::COLOR_BGR2GRAY);

	vector<Point2f> corners;

	bool found = findChessboardCorners(gray_image, board_size, corners);

	if(found == true) {
		raw_image.copyTo(board_visualized_img);

		TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
		cornerSubPix(gray_image, corners, Size(5,5), Size(-1,-1), param);
		cv::drawChessboardCorners(board_visualized_img, board_size, corners, found);
		return true;
	} else {
		return false;
	}
}

void estimate_intrinsic_parameters()
{
	set_img_filenames();

	vector<Point2f> _2d_corners;
	vector<Point3f> _3d_corners;

	for(int i = 0; i < board_size.height; i++) {
		for(int j = 0; j < board_size.width; j++) {
			_3d_corners.push_back(Point3f((float)i * SQUARE_SIZE,
			                              (float)j * SQUARE_SIZE, 0.0f));
		}
	}

	for(int i = 0; i < image_names.size(); i++) {
		Mat image = imread(image_names[i], CV_LOAD_IMAGE_GRAYSCALE);

		findChessboardCorners(image, board_size, _2d_corners);
		TermCriteria param(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1);
		cornerSubPix(image, _2d_corners, Size(5, 5), Size(-1, -1), param);
		if(_2d_corners.size() == board_size.area()) {
			add_board_points(_2d_corners, _3d_corners);
		}
	}

	calibrateCamera(object_3d_points, image_2d_points, image_size,
	                camera_matrix, dist_coeffs, rvecs, tvecs);

	cout << fixed << setprecision(5);

	/* print calibration result to screen */
	cout << "camera matrix:\n";

	for(int i = 0; i < 3; i++) {
		cout << "["  << camera_matrix.at<double>(i, 0)
		     << ", " << camera_matrix.at<double>(i, 1)
		     << ", " << camera_matrix.at<double>(i, 2) << "]\n";
	}

	cout << "distort coefficients:\n";
	cout << "[" << dist_coeffs.at<double>(0, 0)
	     << ", " << dist_coeffs.at<double>(0, 1)
	     << ", " << dist_coeffs.at<double>(0, 2)
	     << ", " << dist_coeffs.at<double>(0, 3)
	     << ", " << dist_coeffs.at<double>(0, 4) << "]\n";

	/* save calibration result into yaml */
	ofstream fout("intrinsic_calibration.yaml");
	fout << "camera_matrix: \n  ["
	     << fixed << setprecision(5);
	for(int i = 0; i < 3; i++) {
		fout << camera_matrix.at<double>(i, 0)
		     << ", " << camera_matrix.at<double>(i, 1)
		     << ", " << camera_matrix.at<double>(i, 2);

		if(i < 2) {
			fout << ",\n  ";
		}
	}
	fout << "]\n";

	fout << "distortion_coefficients: "
	     << "[" << dist_coeffs.at<double>(0, 0)
	     << ", " << dist_coeffs.at<double>(0, 1)
	     << ", " << dist_coeffs.at<double>(0, 2)
	     << ", " << dist_coeffs.at<double>(0, 3)
	     << ", " << dist_coeffs.at<double>(0, 4) << "]";

	cout << "calibration succeeded, press ctrl+c to leave.\n";
}


void undistort_image(const Mat &src, Mat &dst)
{
	Mat map1, map2;

	initUndistortRectifyMap(camera_matrix, dist_coeffs, Mat(), Mat(), image_size, CV_32F, map1, map2);
	remap(src, dst, map1, map2, INTER_LINEAR);
}

void on_click_callback(int event, int x, int y, int flags, void* param)
{
	char img_save_path[100] = "";

	if((event == CV_EVENT_LBUTTONDOWN) && (image_count <= IMG_SAMPLE_SIZE)) {
		if(checkerboard_detected == true) {
			sprintf(img_save_path, "/tmp/intrinsic_%dx%d_%d.jpg", image_width, image_height, image_count);
			imwrite(img_save_path, raw_image);
			cout << "saving image to " << img_save_path << "\n";
			image_count++;
		} else {
			cout << "could not detect checkerboard on image.\n";
		}
	}
}

void ros_thread_entry(void)
{
	ros::spin();
}

void intrinsic_calibration_thread_entry(void)
{
	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");

	namedWindow("intrinsic calibration");
	setMouseCallback("intrinsic calibration", on_click_callback, NULL);

	/* initialize image size */
	ros_cam_dev.read(raw_image);
	image_width = raw_image.cols;
	image_height = raw_image.rows;
	image_size = Size(image_width, image_height);

	cout << "please click the window to save 15 images for calibration.\n";

	Mat undistorted_image;
	vector<Point2f> corners;

	while(image_count <= 15) {
		ros_cam_dev.read(raw_image);

		Mat board_visualized_img;

		if(visualize_checkerboard(raw_image, board_visualized_img) == true) {
			cv::imshow("intrinsic calibration", board_visualized_img);
			checkerboard_detected = true;
		} else {
			checkerboard_detected = false;
			cv::imshow("intrinsic calibration", raw_image);
			checkerboard_detected = false;
		}
		waitKey(30);
	}

	cout << "start estimating extrinsicing parameters...\n";
	estimate_intrinsic_parameters();

	while(1) {
		ros_cam_dev.read(raw_image);

		undistort_image(raw_image, undistorted_image);
		imshow("intrinsic calibration", undistorted_image);
		cv::waitKey(30);
	}

	destroyAllWindows();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vafs");
	ros::Time::init();

	ROSCamDev ros_cam_dev("/arducam/camera/image_raw");

	std::thread thread_ros(ros_thread_entry);
	std::thread thread_intrinsic_calibration(intrinsic_calibration_thread_entry);
	thread_ros.join();
	thread_intrinsic_calibration.join();

	return 0;
}
