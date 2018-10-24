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

void Path_Finder::init(int car_position_x = 100, int car_position_y = 145, int vanishing_point_x = 320, int vanishing_point_y = 235, int num_of_goals = 7) {
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

	for(int i = 0; i < 20; i++){
		dilate(dilatedImg, dilatedImg, Mat());
	}
	
	resize(dilatedImg, dilatedImg, Size(20, 20));
	/* set goals */
	Point goals[100];
	for(int i = 1; i <= num_of_goals; i++) {
		if(i != (num_of_goals+1)/2)
			goals[i-1] = Point(car_position_x + int(60*cos(180/(num_of_goals+1)*i*3.1416/180)), car_position_y - int(60*sin(180/(num_of_goals+1)*i*3.1416/180)));
		else
			goals[i-1] = Point(car_position_x, car_position_y - 60);
	}
	
	/* find path using Astar */
	int costs[100];
	int min_cost = 1000000;
	int min_cost_index = 0;
	
	Astar* astar = new Astar(20, 20);
	astar->load_map(dilatedImg);
	Point start_point(car_position_x/10, car_position_y/10);
	costs[0] = astar->find_path(start_point, Point(goals[num_of_goals/2].x / 10, goals[num_of_goals/2].y / 10));
	min_cost = costs[0];
	for(int i = 1; i <= (int)num_of_goals/2; i++) {
		costs[2*(i-1) + 1] = astar->find_path(start_point, Point(goals[num_of_goals/2 - i].x / 10, goals[num_of_goals/2 - i].y / 10));
	}
	for(int i = 1; i <= (int)num_of_goals/2; i++) {
		costs[2*i] = astar->find_path(start_point, Point(goals[num_of_goals/2 + i].x / 10, goals[num_of_goals/2 + i].y / 10));
	}	
	for(int i = 1; i < num_of_goals; i++) {
		if(min_cost > costs[i]) {
			min_cost = costs[i];
			min_cost_index = i;
		}
	}
	
	printf("min_cost : %d - %d\n", costs[min_cost_index], min_cost_index);
	resize(dilatedImg, dilatedImg, Size(200, 200));
	/* just for imshow */
	cvtColor(remappedImg, remappedImg, CV_GRAY2BGR);
	cvtColor(cannyImg, cannyImg, CV_GRAY2BGR);

	cvtColor(dilatedImg, dilatedImg, CV_GRAY2BGR);
	for(int i = 0; i < num_of_goals; i++) {
		if(i % 2 == 0){
			if(i == min_cost_index){
				circle(dilatedImg, goals[(int)num_of_goals/2 + i/2], 3, Scalar(0, 255, 0), -1);
			}
			else{
				circle(dilatedImg, goals[(int)num_of_goals/2 + i/2], 3, Scalar(35, 125, 255), -1);
			}
		}
		else {
			if(i == min_cost_index){
				circle(dilatedImg, goals[(int)num_of_goals/2 - (i+1)/2], 3, Scalar(0, 255, 0), -1);
			}
			else{
				circle(dilatedImg, goals[(int)num_of_goals/2 - (i+1)/2], 3, Scalar(25, 125, 255), -1);
			}
		}
	}
	circle(dilatedImg, Point(car_position_x, car_position_y), 3, Scalar(35, 250, 255), -1);
	
	
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