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
#include "apriltag_pose.h"
}

using namespace std;
using namespace cv;

void tags_visualize(cv::Mat& frame, zarray_t* detections)
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

bool tag_localize(int id, zarray_t* detections, apriltag_detection_info_t& tag_info, apriltag_pose_t& pose)
{
	apriltag_detection_t *det;

	for(int i = 0; i < zarray_size(detections); i++) {
		zarray_get(detections, i, &det);
		tag_info.det = det;

		if(det->id != id) {
			continue;
		}
		//XXX: how to handle more than one tag with same id?

		double err = estimate_tag_pose(&tag_info, &pose);

		return true; //tag exists
	}

	return false; //tag not exist
}

int main(void)
{
	/* camera initialization */
	VideoCapture camera(0);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_IMAGE_WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_IMAGE_HEIGHT);
	if (!camera.isOpened()) {
		cerr << "couldn't open video camerature device" << endl;
		return -1;
	}

	/* camera parameters */
	float fx = 656.24987;
	float fy = 656.10660;
	float cx = 327.36105;
	float cy = 240.03464;

	/* infomation of tag to be detected */
	float tag_size = 0.1; //[m]
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

	Mat frame, gray;
	while (true) {
		camera >> frame;
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

	return 0;
}

