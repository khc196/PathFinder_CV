#ifndef PATH_FINDER_H
#define PATH_FINDER_H

#include <iostream>
#include <fstream>
#include <ctime>
#include <queue>
#include <cv.h>
#include <unistd.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <string.h>
#include <sys/time.h>
#include "inverseMapping.hpp"
#include "Astar.h"
using namespace cv;
using namespace std;


string to_string(int n) {
	stringstream s;
	s << n;
	return s.str();
}

class Path_Finder {
protected:
	Mat cannyImg, grayImg, remappedImg, dilatedImg;
    int vanishing_point_x;
    int vanishing_point_y;
    int* ipm_table;
	VideoWriter outputVideo;
	bool isinit = true;
	float steer;
	int num_of_goals;

	int car_position_x;
	int car_position_y;
public:
	Path_Finder() {}
	void init(int, int, int, int, int);
	void operate(Mat originImg);
};

void Path_Finder::init(int car_position_x = 100, int car_position_y = 145, int vanishing_point_x = 320, int vanishing_point_y = 235, int num_of_goals = 5) {
	string path = "output.avi";
	struct tm* datetime;
	time_t t;
	t = time(NULL);
	datetime = localtime(&t);
	string s_t = path;
	
	this->vanishing_point_x = vanishing_point_x;
	this->vanishing_point_y = vanishing_point_y;
	this->car_position_x = car_position_x;
	this->car_position_y = car_position_y;
	steer = 0.f;
	//outputVideo.open(s_t, VideoWriter::fourcc('X', 'V', 'I', 'D'), 10, Size(720, 120), true);
	outputVideo.open(s_t, VideoWriter::fourcc('D', 'I', 'V', 'X'), 10, Size(920, 200), true);
	ipm_table = new int[DST_REMAPPED_WIDTH * DST_REMAPPED_HEIGHT];
    build_ipm_table(SRC_RESIZED_WIDTH, SRC_RESIZED_HEIGHT, 
                    DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, 
                    vanishing_point_x, vanishing_point_y, ipm_table);
	this->num_of_goals = num_of_goals;
}
void Path_Finder::operate(Mat originImg) {
	Mat grayImg;
    Mat remappedImg = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);
	//remappedImg = remappedImg(Rect(0, 0, 200, 200));
	cvtColor(originImg, grayImg, CV_BGR2GRAY);
    inverse_perspective_mapping(DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, grayImg.data, ipm_table, remappedImg.data);
	Canny(remappedImg, cannyImg, 70, 210);
	morphologyEx(cannyImg, dilatedImg, MORPH_CLOSE, Mat(12,12, CV_8U, Scalar(1)));
	Mat black(Size(200, 70), CV_8U, Scalar(0));
	black.copyTo(dilatedImg(Rect(0, 130, 200, 70)));
	
	const float PI = 3.1416f;
	vector<cv::Vec2f> lines;
	HoughLines(dilatedImg, lines, 1, PI/180, 100);

	vector<Vec2f>::const_iterator it= lines.begin();
	
	while (it!=lines.end()) {
		float rho = (*it)[0];   
		float theta = (*it)[1]; 
		if (theta < PI/4. || theta > 3.*PI/4.) { 
			Point pt1(rho/cos(theta), 0); 
			Point pt2((rho-dilatedImg.rows*sin(theta))/cos(theta), dilatedImg.rows);
			line(dilatedImg, pt1, pt2, Scalar(0), 10); 

		} else {
			Point pt1(0,rho/sin(theta)); 
			Point pt2(dilatedImg.cols,(rho-dilatedImg.cols*cos(theta))/sin(theta));
			line(dilatedImg, pt1, pt2, Scalar(0), 10);
		}
		++it;
	}
	Mat roiCircle(Size(200, 200), CV_8U, Scalar(0));
	circle(roiCircle, Point(car_position_x, car_position_y), 50, Scalar(255), -1);
	dilatedImg = dilatedImg&roiCircle;
	for(int i = 0; i < 50; i++){
		dilate(dilatedImg, dilatedImg, Mat());
	}
	
	/* just for imshow */
	cvtColor(remappedImg, remappedImg, CV_GRAY2BGR);
	cvtColor(cannyImg, cannyImg, CV_GRAY2BGR);

	cvtColor(dilatedImg, dilatedImg, CV_GRAY2BGR);
	
	Mat result;
	resize(originImg, originImg, Size(320, 200));
	resize(remappedImg, remappedImg, Size(200, 200));

	hconcat(originImg, remappedImg, result);
	resize(cannyImg, cannyImg, Size(200, 200));
	hconcat(result, cannyImg, result);
	resize(dilatedImg, dilatedImg, Size(200, 200));
	hconcat(result, dilatedImg, result);
	
	imshow("result", result);
	//outputVideo << result;
	if(waitKey(10) == 0) {
		return;
	}
	return;
}
#endif