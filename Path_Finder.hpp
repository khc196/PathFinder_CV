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
using namespace cv;
using namespace std;

string to_string(int n) {
	stringstream s;
	s << n;
	return s.str();
}

class Path_Finder {
protected:
	Mat cannyImg, grayImg, remappedImg;

    int vanishing_point_x;
    int vanishing_point_y;
    int* ipm_table;
	VideoWriter outputVideo;
	bool isinit = true;
public:
	Path_Finder() {}
	void init();
	void operate(Mat originImg);
	queue<float> direction_vec;
};

void Path_Finder::init() {
	string path = "output.avi";
	struct tm* datetime;
	time_t t;
	t = time(NULL);
	datetime = localtime(&t);
	string s_t = path;

	vanishing_point_x = 320;
	vanishing_point_y = 235;

	//outputVideo.open(s_t, VideoWriter::fourcc('X', 'V', 'I', 'D'), 10, Size(720, 120), true);
	outputVideo.open(s_t, VideoWriter::fourcc('D', 'I', 'V', 'X'), 10, Size(720, 120), true);
	ipm_table = new int[DST_REMAPPED_WIDTH * DST_REMAPPED_HEIGHT];
    build_ipm_table(SRC_RESIZED_WIDTH, SRC_RESIZED_HEIGHT, 
                    DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, 
                    vanishing_point_x, vanishing_point_y, ipm_table);
}
void Path_Finder::operate(Mat originImg) {
	Mat grayImg;
    Mat remappedImg = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);
	//remappedImg = remappedImg(Rect(0, 0, 200, 200));
	cvtColor(originImg, grayImg, CV_BGR2GRAY);
    inverse_perspective_mapping(DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, grayImg.data, ipm_table, remappedImg.data);
	Canny(remappedImg, cannyImg, 70, 210);
	Mat dilatedImg; 
	morphologyEx(cannyImg, dilatedImg, MORPH_CLOSE, Mat(12,12, CV_8U, Scalar(1)));

	/* control */
	int car_position_x = 100;
	int car_position_y = 150;

	// from bottom
	vector<Point> vec_cp;
	int left_wall = 0, right_wall = 199;
	bool start_scan_l = true;
	bool start_scan_r = true;
	for(int i = car_position_y; i >= 0; i-=7){
		// left side
		for(int j = car_position_x-1; j >= 0; j--){
			if(dilatedImg.data[i * dilatedImg.step + j] > 0){
				left_wall = j;
				break;
			}
		}
		// right side
		for(int j = car_position_x+1; j < 200; j++){
			if(dilatedImg.data[i * dilatedImg.step + j] > 0){
				right_wall = j;
				break;
			}
		}
		int center_point_x = (left_wall + right_wall) / 2;
		vec_cp.push_back(Point(center_point_x, -i));
		if(vec_cp.size() > 5)
			break;
	}
	cvtColor(remappedImg, remappedImg, CV_GRAY2BGR);
	cvtColor(cannyImg, cannyImg, CV_GRAY2BGR);
	cvtColor(dilatedImg, dilatedImg, CV_GRAY2BGR);
	for(int i = 0; i < vec_cp.size() - 1; i++){
		Point vec = Point(vec_cp[i+1].x - vec_cp[i].x, vec_cp[i+1].y - vec_cp[i].y); 
		line(dilatedImg, Point(vec_cp[i+1].x, -vec_cp[i+1].y), Point(vec_cp[i].x, -vec_cp[i].y), Scalar(255,0,0), 1, CV_AA);
		float steer;
		steer = 0.f;
		if(vec.x != 0)
			steer = -(atan(-vec.y / vec.x) * 180 / M_PI) / 90; 
		printf("steer : %f\n", steer);
		direction_vec.push(steer);
	}

	/* just for imshow */
	Mat result;
	resize(originImg, originImg, Size(320, 200));
	resize(remappedImg, remappedImg, Size(200, 200));
	hconcat(originImg, remappedImg, result);
	resize(cannyImg, cannyImg, Size(200, 200));
	hconcat(result, cannyImg, result);
	resize(dilatedImg, dilatedImg, Size(200, 200));
	hconcat(result, dilatedImg, result);
	imshow("result", result);
	if(waitKey(10) == 0) {
		return;
	}
	return;
}
#endif