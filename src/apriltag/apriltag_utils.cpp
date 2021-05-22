#include <iostream>
#include "opencv2/opencv.hpp"

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

