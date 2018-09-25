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
	Mat cannyImg, grayImg, remappedImg, dilatedImg;
	int visit_array[200][200];
	int ds_array[200][200];
    int vanishing_point_x;
    int vanishing_point_y;
    int* ipm_table;
	VideoWriter outputVideo;
	bool isinit = true;
	float steer;
	const int car_position_x = 100;
	const int car_position_y = 160;
	const int car_width = 70;
	const int car_height = 40;
	const int straight_unit = 10;
	const int curve_unit = 5;
	int find_path(int x, int y, int count);
	int find_init_position();
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
	steer = 0.f;
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
	morphologyEx(cannyImg, dilatedImg, MORPH_CLOSE, Mat(12,12, CV_8U, Scalar(1)));

	/* control */
	for(int i = 0; i < 200; i++){
		for(int j = 0; j < 200; j++){
			visit_array[i][j] = -1;
			ds_array[i][j] = -1;
		}
	}
	int init_position_x = find_init_position();
	find_path(init_position_x, car_position_y, 0);
	
	cvtColor(remappedImg, remappedImg, CV_GRAY2BGR);
	cvtColor(cannyImg, cannyImg, CV_GRAY2BGR);
	cvtColor(dilatedImg, dilatedImg, CV_GRAY2BGR);
	int current_x = init_position_x;
	int current_y = car_position_y;
	vector<Point> way_points;

	while(current_y > 30){
		circle(dilatedImg, Point(current_x, current_y), 3, Scalar(0, 255, 0), -1);
		way_points.push_back(Point(current_x, current_y));
		if(ds_array[current_x][current_y] == -1){
			break;
		}
		switch(ds_array[current_x][current_y]){
			case 0:
			current_y -= straight_unit;
			break;
			case 1:
			current_x -= curve_unit;
			current_y -= curve_unit;
			break;
			case 2:
			current_x += curve_unit;
			current_y -= curve_unit;
			break;
		}
	}
	
	for(int i = 0; i < way_points.size() - 1; i++){
		Point vec = Point(way_points[i+1].x - way_points[i].x, way_points[i+1].y - way_points[i].y); 
		line(dilatedImg, Point(way_points[i+1].x, way_points[i+1].y), Point(way_points[i].x, way_points[i].y), Scalar(0,255,0), 1, CV_AA);
		//rectangle(dilatedImg, Point(vec_cp[i].x - car_width/2, -vec_cp[i].y + car_height)
		
		steer = 0.f;
		if(vec.x != 0)
			steer = (atan(-vec.y / vec.x) * 180 / M_PI) / 90; 
		printf("steer : %f\n", steer);
		direction_vec.push(steer);
	}
	printf("---------------------------------\n");
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
int Path_Finder::find_init_position(){
	int min_cost = 1000;
	int position_x;
	for(int i = car_position_x; i < 200; i++){
		int count = 0;
		for(int j = i - car_width/2; j <= i + car_width/2; j++){
			for(int k = car_position_y; k >= car_position_y - car_height/2; k--){
				if(j > 0 && dilatedImg.data[k * dilatedImg.step + j] > 0){
					count++;
				}
			}
		}
		if(min_cost > count){
			min_cost = count;
			position_x = i;
		}
	}
	for(int i = car_position_x; i >= 0; i--){
		int count = 0;
		for(int j = i - car_width/2; j <= i + car_width/2; j++){
			for(int k = car_position_y; k >= car_position_y - car_height/2; k--){
				if(j < 200 && dilatedImg.data[k * dilatedImg.step + j] > 0){
					count++;
				}
			}
		}
		if(min_cost > count){
			min_cost = count;
			position_x = i;
		}
	}
	return position_x;
}
int Path_Finder::find_path(int x, int y, int count){
	if (visit_array[x][y] != -1) {
		return visit_array[x][y];
	}
	visit_array[x][y] = 0;
	for(int j = x - car_width/2; j <= x + car_width/2; j++){
		for(int k = y; k >= y - car_height/2; k--){
			if(j >= 0 && j < 200 &&dilatedImg.data[k * dilatedImg.step + j] > 0){
				visit_array[x][y]++;
			}
		}
	}
	if (y <= 30 || count > 10 || visit_array[x][y] > 200) {
		return visit_array[x][y];
	}
	//printf("%d %d, %d\n", x, y, visit_array[x][y]);
	int up_cost = 10000, left_cost = 10000, right_cost = 10000;
	up_cost = find_path(x, y-straight_unit, count+1);
	if(x-curve_unit >= 0){
		left_cost = find_path(x-curve_unit, y-curve_unit, count+1);
	}
	if(x+curve_unit < 200){
		right_cost = find_path(x+curve_unit, y-curve_unit, count+1);
	}
	int min_cost = up_cost;
	ds_array[x][y] = 0;
	if (min_cost > left_cost){
		min_cost = left_cost;
		ds_array[x][y] = 1;
	}
	if (min_cost > right_cost){
		min_cost = right_cost;
		ds_array[x][y] = 2;
	}
	
	return min_cost + visit_array[x][y];
}
#endif