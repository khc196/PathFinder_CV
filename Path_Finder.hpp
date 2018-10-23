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
	
	// for pid
	const float KP = -0.007;
	const float KI = -0.00001;
	const float KD = -0.0001;
	float position;
	float error;
	float prev_error;
	float error_sum;
	
	float KP2 = 0.15f;
	float KI2 = 0.001f;
	float KD2 = 0.001f;
	float error2;
	float prev_error2;
	float error_sum2;
	int DI;
	int GB;
	int ST;

	const int car_position_x = 100;
	const int car_position_y = 145;
	const int car_width = 4;
	const int car_height = 4;
	int straight_unit = 10;
	int curve_unit = 4;
	int prev_position_x = 100;;
	int find_path(int x, int y, int count, int direction);
public:
	Path_Finder() {}
	void init();
	void operate(Mat originImg);
	queue<float> direction_vec;
	int speed;
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
	outputVideo.open(s_t, VideoWriter::fourcc('D', 'I', 'V', 'X'), 10, Size(720, 200), true);
	ipm_table = new int[DST_REMAPPED_WIDTH * DST_REMAPPED_HEIGHT];
    build_ipm_table(SRC_RESIZED_WIDTH, SRC_RESIZED_HEIGHT, 
                    DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, 
                    vanishing_point_x, vanishing_point_y, ipm_table);
	position = 0.f;
	error = 0.f;
	prev_error = 0.f;
	error_sum = 0.f;
	error2 = 0.f;
	prev_error2 = 0.f;
	error_sum2 = 0.f;

	speed = 120;
}
void Path_Finder::operate(Mat originImg) {
	ifstream pid_file("pid_K");
	pid_file >> KP2 >> KI2 >> KD2 >> DI >> GB >> ST;
	pid_file.close(); 	
	Mat grayImg;
    Mat remappedImg = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);
	cvtColor(originImg, grayImg, CV_BGR2GRAY);
    inverse_perspective_mapping(DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, grayImg.data, ipm_table, remappedImg.data);
	
	Canny(remappedImg, cannyImg, 50, 150);
	morphologyEx(cannyImg, dilatedImg, MORPH_CLOSE, Mat(12,12, CV_8U, Scalar(1)));
	
	Mat black(Size(200, 70), CV_8U, Scalar(0));
	black.copyTo(dilatedImg(Rect(0, 130, 200, 70)));

	// vector<Vec2f> lines;
	// Mat linemask(Size(200, 200), CV_8U, Scalar(0));
 	// cv::HoughLines(dilatedImg, lines, 1,3.14/180, GB);
	// vector<Vec2f>::const_iterator it= lines.begin();
	// while (it!=lines.end()) {
	// 	float rho = (*it)[0];   // 첫 번째 요소는 rho 거리
	// 	float theta = (*it)[1]; // 두 번째 요소는 델타 각도
	// 	if (theta < 3.14/4. || theta > 3.*3.14/4.) { // 수직 행
	// 		Point pt1(rho/cos(theta), 0); // 첫 행에서 해당 선의 교차점   
	// 		Point pt2((rho-cannyImg.rows*sin(theta))/cos(theta), cannyImg.rows);
	// 		line(linemask, pt1, pt2, Scalar(255), 3); // 하얀 선으로 그리기
	// 	} 
	// 	else { // 수평 행
	// 		Point pt1(0,rho/sin(theta)); // 첫 번째 열에서 해당 선의 교차점  
	// 		Point pt2(cannyImg.cols,(rho-cannyImg.cols*cos(theta))/sin(theta));
	// 		line(linemask, pt1, pt2, Scalar(255), 3); // 하얀 선으로 그리기
	// 	}
	// 	++it;
	// }
	
	// dilatedImg = dilatedImg & linemask;
	straight_unit = ST * 2;
	curve_unit = straight_unit;
	for(int i = 0; i < DI; i++){
		dilate(dilatedImg, dilatedImg, Mat());
	}

	/* control */
	for(int i = 0; i < 200; i++){
		for(int j = 0; j < 200; j++){
			visit_array[i][j] = -1;
			ds_array[i][j] = -1;
		}
	}
	int init_position_x = car_position_x;
	find_path(init_position_x, car_position_y, 0, 0);
	
	cvtColor(remappedImg, remappedImg, CV_GRAY2BGR);
	cvtColor(cannyImg, cannyImg, CV_GRAY2BGR);
	cvtColor(dilatedImg, dilatedImg, CV_GRAY2BGR);
	int current_x = init_position_x;
	int current_y = car_position_y;
	vector<Point> way_points;

	while(current_y > 30){
		//circle(dilatedImg, Point(current_x, current_y), 3, Scalar(0, 255, 0), -1);
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
			case 3:
			current_x += curve_unit;
			current_y -= straight_unit;
			case 4:
			current_x += curve_unit;
			current_y -= straight_unit;
			break;
		}
	}
	prev_error = error;
	position = (float)init_position_x;
	error = 100.f - init_position_x;
	error_sum += error;
	prev_error2 = error2;
	
	float pid = error * KP + error_sum * KI + (error - prev_error) * KD;
	steer = 0.f;
	
	printf("KP : %f, KI : %f, KD : %f\n", KP2, KI2, KD2);
	KP2 /= 10000;
	KI2 /= 10000;
	KD2 /= 10000;
	for(int i = 0; i < way_points.size() - 1; i++){
		Point vec = Point(way_points[i+1].x - way_points[i].x, way_points[i+1].y - way_points[i].y); 
		line(dilatedImg, Point(way_points[i+1].x, way_points[i+1].y), Point(way_points[i].x, way_points[i].y), Scalar(0,255,0), 10, CV_AA);
		//line(cannyImg, Point(way_points[i+1].x, way_points[i+1].y), Point(way_points[i].x, way_points[i].y), Scalar(0,255,0), 10, CV_AA);
		
		if(vec.x != 0){
			prev_error2 = error2;
			error2 = (atan(-vec.y / vec.x) * 180 / M_PI) / 90;
			error_sum2 += error2;
			if (error_sum2 > 1.0f) error_sum2 = 1.0f;
			else if(error_sum2 < -1.0f) error_sum2 = -1.0f;
			float pid2 = error2 * KP2 + error_sum2 * KI2 + (error2 - prev_error2) * KD2;
			steer += pid2;
		}
		else{
			error2 = 0;
			prev_error2 = 0;
			error_sum2 = 0;
		}
		printf("steer : %f\n", steer);
		direction_vec.push(steer);
	}
	speed = 83.f;
	//speed = 83.f - abs((float)current_x - 100.f) / 3.5f;
	printf("---------------------------------\n");
	/* just for imshow */
	Mat result;
	resize(originImg, originImg, Size(320, 200));
	resize(remappedImg, remappedImg, Size(200, 200));

	Mat rewarp = originImg.clone();
	Point2f inputQuad[4]; 
    Point2f outputQuad[4];
	Mat mask;
	inRange(dilatedImg, Scalar(10,10,10), Scalar(255,255,255), mask);
	inputQuad[0] = Point2f(0, 0);
    inputQuad[1] = Point2f(199, 0);
    inputQuad[2] = Point2f(199, 199);
    inputQuad[3] = Point2f(0, 199);  
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = Point2f(100,115 );
    outputQuad[1] = Point2f(209,115);
    outputQuad[2] = Point2f(1200,450);
    outputQuad[3] = Point2f(-850,450);
	Mat lambda = getPerspectiveTransform(inputQuad, outputQuad);
	
	Mat red = dilatedImg.clone();
	red.setTo(Scalar(0,0,0), mask);
	warpPerspective(red, rewarp, lambda, Size(320, 200));
	addWeighted(rewarp, 0.4, originImg, 1.0, 0.0, originImg);

	//dilatedImg = dilatedImg(Rect(0, 30, 200, 105));
	//resize(dilatedImg, dilatedImg, Size(200, 200));
	hconcat(originImg, cannyImg, originImg);
	hconcat(originImg, dilatedImg, originImg);
	imshow("result", originImg);
	//outputVideo << originImg;
	if(waitKey(10) == 0) {
		return;
	}
	return;
}

int Path_Finder::find_path(int x, int y, int count, int direction){
	if (visit_array[x][y] != -1) {
		return visit_array[x][y];
	}	
	visit_array[x][y] = 0;
	for(int j = x - car_width/2; j <= x + car_width/2; j++){
		for(int k = y; k >= y - car_height/2; k--){
			if(j >= 0 && j < 200 &&dilatedImg.data[k * dilatedImg.step + j] > 0){
				visit_array[x][y] += dilatedImg.data[k * dilatedImg.step + j];
			}
		}
	}
	if(visit_array[x][y] > 10){
		visit_array[x][y] *= 10;
	}
	if (y <= 30 || x <= 10 || x >= 190) {
		return visit_array[x][y];
	}
	//printf("%d %d, %d\n", x, y, visit_array[x][y]);
	int cost1 = 100000, cost2 = 100000, cost3 = 100000, cost4 = 100000, cost5 = 100000;
	int up_cost, left_cost, right_cost, semi_left_cost, semi_right_cost;
	int cost_direction[5];
	up_cost = visit_array[x][y] + find_path(x, y-straight_unit, count+1, 0);
	if(x-curve_unit >= 0){
		semi_left_cost = visit_array[x][y] + find_path(x-curve_unit, y-straight_unit, count+1, 1);
		left_cost = visit_array[x][y] + find_path(x-curve_unit, y-curve_unit, count+1, 3);
	}
	if(x+curve_unit < 200){
		semi_right_cost = visit_array[x][y] + find_path(x+curve_unit, y-straight_unit, count+1, 2);
		right_cost = visit_array[x][y] + find_path(x+curve_unit, y-curve_unit, count+1, 4);
	}
	if(x-curve_unit >= 0){
		
	}
	if(x+curve_unit < 200){
		
	}
	switch(direction){
		case 0:
		cost1 = up_cost;
		cost2 = semi_left_cost;
		cost3 = semi_right_cost;
		cost4 = left_cost;
		cost5 = right_cost;
		cost_direction[0] = 0;
		cost_direction[1] = 1;
		cost_direction[2] = 2;
		cost_direction[3] = 3;
		cost_direction[4] = 4;
		break;
		case 1:
		cost1 = semi_left_cost;
		cost2 = left_cost;
		cost3 = up_cost;
		cost4 = semi_right_cost;
		cost5 = right_cost;
		cost_direction[0] = 1;
		cost_direction[1] = 3;
		cost_direction[2] = 0;
		cost_direction[3] = 2;
		cost_direction[4] = 4;
		break;
		case 2:
		cost1 = semi_right_cost;
		cost2 = up_cost;
		cost3 = right_cost;
		cost4 = semi_left_cost;
		cost5 = left_cost;
		cost_direction[0] = 2;
		cost_direction[1] = 0;
		cost_direction[2] = 4;
		cost_direction[3] = 1;
		cost_direction[4] = 3;
		break;
		case 3:
		cost1 = left_cost;
		cost2 = semi_left_cost;
		cost3 = up_cost;
		cost4 = semi_right_cost;
		cost5 = right_cost;
		cost_direction[0] = 3;
		cost_direction[1] = 1;
		cost_direction[2] = 0;
		cost_direction[3] = 2;
		cost_direction[4] = 4;
		break;
		case 4:
		cost1 = right_cost;
		cost2 = semi_right_cost;
		cost3 = up_cost;
		cost4 = semi_left_cost;
		cost5 = left_cost;
		cost_direction[0] = 4;
		cost_direction[1] = 2;
		cost_direction[2] = 0;
		cost_direction[3] = 1;
		cost_direction[4] = 3;
		break;
	}


	int min_cost = cost1;
	ds_array[x][y] = cost_direction[0];
	
	if (min_cost > cost2){
		min_cost = cost2;
		ds_array[x][y] = cost_direction[1];
	}
	if (min_cost > cost3){
		min_cost = cost3;
		ds_array[x][y] = cost_direction[2];
	}
	if (min_cost > cost4){
		min_cost = cost4;
		ds_array[x][y] = cost_direction[3];
	}
	if (min_cost > cost5){
		min_cost = cost5;
		ds_array[x][y] = cost_direction[4];
	}
	visit_array[x][y] = min_cost;
	return min_cost;
}
#endif