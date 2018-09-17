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
	Mat cannyImg;

    int vanishing_point_x;
    int vanishing_point_y;
    int* ipm_table;
	VideoWriter outputVideo;
	bool isinit = true;
public:
	Path_Finder() {}
	void init();
	int operate(Mat originImg);
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
int Path_Finder::operate(Mat originImg) {
	Mat imgray;
    Mat imremapped = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);
	//imremapped = imremapped(Rect(0, 0, 200, 200));
	cvtColor(originImg, imgray, CV_BGR2GRAY);
    inverse_perspective_mapping(DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, imgray.data, ipm_table, imremapped.data);
	Canny(imremapped, cannyImg, 70, 210);
	Mat dilatedImg;
	morphologyEx(cannyImg, dilatedImg, MORPH_CLOSE, Mat(9,9, CV_8U, Scalar(1)));
	Mat result;
	cvtColor(imremapped, imremapped, CV_GRAY2BGR);
	cvtColor(cannyImg, cannyImg, CV_GRAY2BGR);
	cvtColor(dilatedImg, dilatedImg, CV_GRAY2BGR);
	resize(originImg, originImg, Size(480, 320));
	resize(imremapped, imremapped, Size(320, 320));
	hconcat(originImg, imremapped, result);
	resize(cannyImg, cannyImg, Size(320, 320));
	hconcat(result, cannyImg, result);
	resize(dilatedImg, dilatedImg, Size(320, 320));
	hconcat(result, dilatedImg, result);
	imshow("result", result);
	if(waitKey(10) == 0) {
		return -1;
	}
	return 0;
}
#endif