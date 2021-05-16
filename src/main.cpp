#include <iostream>
#include "opencv2/opencv.hpp"
#include "camera_config.hpp"

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
}

using namespace std;
using namespace cv;

void tag_visualize(cv::Mat& frame, zarray_t* detections)
{
	//draw detection outlines
	for (int i = 0; i < zarray_size(detections); i++) {
		apriltag_detection_t *det;
		zarray_get(detections, i, &det);
		line(frame, Point(det->p[0][0], det->p[0][1]),
		     Point(det->p[1][0], det->p[1][1]),
		     Scalar(0, 0xff, 0), 2);
		line(frame, Point(det->p[0][0], det->p[0][1]),
		     Point(det->p[3][0], det->p[3][1]),
		     Scalar(0, 0, 0xff), 2);
		line(frame, Point(det->p[1][0], det->p[1][1]),
		     Point(det->p[2][0], det->p[2][1]),
		     Scalar(0xff, 0, 0), 2);
		line(frame, Point(det->p[2][0], det->p[2][1]),
		     Point(det->p[3][0], det->p[3][1]),
		     Scalar(0xff, 0, 0), 2);

		stringstream ss;
		ss << det->id;
		String text = ss.str();
		int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
		double fontscale = 1.0;
		int baseline;
		Size textsize = getTextSize(text, fontface, fontscale, 2,
		                            &baseline);
		putText(frame, text, Point(det->c[0]-textsize.width/2,
		                           det->c[1]+textsize.height/2),
		        fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
	}
}

int main(void)
{
	//initialize camera
	VideoCapture camera(0);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_IMAGE_WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_IMAGE_HEIGHT);
	if (!camera.isOpened()) {
		cerr << "couldn't open video camerature device" << endl;
		return -1;
	}

	apriltag_family_t *tf = NULL;
	tf = tag36h11_create();
	//tf = tag25h9_create();
	//tf = tag16h5_create();
	//tf = tagCircle21h7_create();
	//tf = tagCircle49h12_create();
	//tf = tagStandard41h12_create();
	//tf = tagStandard52h13_create();
	//tf = tagCustom48h12_create();

	apriltag_detector_t *td = apriltag_detector_create();
	apriltag_detector_add_family(td, tf);
	td->quad_decimate = 2.0f;
	td->quad_sigma = 0.0f;
	td->nthreads = 1;
	td->debug = 0;
	td->refine_edges = 1;

	Mat frame, gray;
	while (true) {
		camera >> frame;
		cvtColor(frame, gray, COLOR_BGR2GRAY);

		//make an image_u8_t header for the mat data
		image_u8_t im = { .width = gray.cols,
		                  .height = gray.rows,
		                  .stride = gray.cols,
		                  .buf = gray.data
		                };

		zarray_t *detections = apriltag_detector_detect(td, &im);

		tag_visualize(frame, detections);

		apriltag_detections_destroy(detections);

		imshow("Tag Detections", frame);
		if (waitKey(30) >= 0) {
			break;
		}
	}

	apriltag_detector_destroy(td);

	tag36h11_destroy(tf);
	//tag25h9_destroy(tf);
	//tag16h5_destroy(tf);
	//tagCircle21h7_destroy(tf);
	//tagCircle49h12_destroy(tf);
	//tagStandard41h12_destroy(tf);
	//tagStandard52h13_destroy(tf);
	//tagCustom48h12_destroy(tf);

	return 0;
}

