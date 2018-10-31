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
#include <cmath>
#include "inverseMapping.hpp"
#include "Astar.h"
#include "polyfit.hpp"
using namespace cv;
using namespace std;


string to_string(int n) {
	stringstream s;
	s << n;
	return s.str();
}
float euclideanDistance(Point center, Point point) {
	return sqrt(pow((center.x - point.x),2) + pow((center.y - point.y), 2));
}
double frand() {
    return 2*((rand()/(double)RAND_MAX) - 0.5);
}

class Path_Finder {
protected:
	Mat cannyImg, grayImg, remappedImg, dilatedImg;
    int vanishing_point_x;
    int vanishing_point_y;
    int* ipm_table;
	VideoWriter outputVideo;
	bool isinit = true;
	int num_of_goals;

	int car_position_x;
	int car_position_y;
	vector<double> prev_line;

	/* for Kalman */
	//initial values for the kalman filter
	float x_est_last = 0;
	float P_last = 0;
	//the noise in the system
	float Q = 0.002;
	float R = 0.2;
	
	float K;
	float P;
	float P_temp;
	float x_temp_est;
	float x_est;
	float z_measured; //the 'noisy' value we measured
	float z_real = 100; //the ideal value we wish to measure	
public:
	Path_Finder() {}
	void init(int, int, int, int, int);
	void operate(Mat originImg);
	float prev_error, error_sum;
	float steer;
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
	outputVideo.open(s_t, VideoWriter::fourcc('D', 'I', 'V', 'X'), 10, Size(1120, 200), true);
	ipm_table = new int[DST_REMAPPED_WIDTH * DST_REMAPPED_HEIGHT];
    build_ipm_table(SRC_RESIZED_WIDTH, SRC_RESIZED_HEIGHT, 
                    DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, 
                    vanishing_point_x, vanishing_point_y, ipm_table);
	this->num_of_goals = num_of_goals;
	prev_line.push_back(0.f);
	prev_line.push_back(0.f);
	srand(0);
	x_est_last = z_real + frand()*0.09;
}


void Path_Finder::operate(Mat originImg) {
	Mat grayImg;
    Mat remappedImg = Mat(DST_REMAPPED_HEIGHT, DST_REMAPPED_WIDTH, CV_8UC1);
	//remappedImg = remappedImg(Rect(0, 0, 200, 200));
	cvtColor(originImg, grayImg, CV_BGR2GRAY);
    inverse_perspective_mapping(DST_REMAPPED_WIDTH, DST_REMAPPED_HEIGHT, grayImg.data, ipm_table, remappedImg.data);

	Canny(remappedImg, cannyImg, 50, 150);
	morphologyEx(cannyImg, dilatedImg, MORPH_CLOSE, Mat(12,12, CV_8U, Scalar(1)));
	Mat black(Size(200, 70), CV_8U, Scalar(0));
	black.copyTo(dilatedImg(Rect(0, 130, 200, 70)));
	
	const float PI = 3.1416f;
	vector<cv::Vec2f> lines;
	HoughLines(dilatedImg, lines, 1, PI/180, 85);

	vector<Vec2f>::const_iterator it= lines.begin();

	while (it!=lines.end()) {
		float rho = (*it)[0];   
		float theta = (*it)[1]; 
		if (theta < PI/4. || theta > 3.*PI/4.) { 
			Point pt1(rho/cos(theta), 0); 
			Point pt2((rho-dilatedImg.rows*sin(theta))/cos(theta), dilatedImg.rows);
			line(dilatedImg, pt1, pt2, Scalar(0), 10); 

		} 
		else {
			Point pt1(0,rho/sin(theta)); 
			Point pt2(dilatedImg.cols,(rho-dilatedImg.cols*cos(theta))/sin(theta));
			line(dilatedImg, pt1, pt2, Scalar(0), 10);
		}
		++it;
	}
	// for(int i = 0; i < 200; i++){
	// 	for(int j = 0; j < 200; j++){
	// 		if(remappedImg.data[i * remappedImg.step + j] < 90){
	// 			dilatedImg.data[i * dilatedImg.step + j] = 0;
	// 		}
	// 	}
	// }
	
	Mat roiCircle(Size(200, 200), CV_8U, Scalar(0));
	ellipse(roiCircle, Point(car_position_x, car_position_y), Size(70, 120), 0.0, 0.0, 360.0, Scalar(255), -1);
	dilatedImg = dilatedImg&roiCircle;
	erode(dilatedImg, dilatedImg, Mat());
	GaussianBlur(dilatedImg, dilatedImg, Size(3, 3), 1.5);
	vector<Point> nonZeros;
	for(int i = 0; i < 200; i++){
		for(int j = 0; j < 200; j++){
			if(dilatedImg.data[i * dilatedImg.step + j] > 0){
				nonZeros.push_back(Point(j, i));
			}
		}
	}
	int degree = 1;
	vector<double> poly_line = polyfit(nonZeros, degree);
	Mat detectImg(Size(200, 200), CV_8U, Scalar(0));
	cvtColor(detectImg, detectImg, CV_GRAY2BGR);
	for(int i = 0; i < 200; i++){
		int y = 0;
		for(int j = 0; j <= degree; j++)
			y += poly_line[j] * pow(i, j);
		if(y >= 0 && y <= 140){
			circle(detectImg, Point(i, y), 5, Scalar(0, 255, 0), -1);
		}
	}
	ifstream inFile("pid_K");
	float KP, KI, KD;
	inFile >> KP >> KI >> KD;
	KP /= 10000;
	KI /= 10000;
	KD /= 10000;
    inFile.close();
	float position, error;
	if(!isnan(poly_line[1])){
		position = (140 - poly_line[0]) / poly_line[1];

		// x_temp_est = x_est_last;
		// P_temp = P_last + Q;
        // //calculate the Kalman gain
        // K = P_temp * (1.0/(P_temp + R));
        // //measure
        // z_measured = position; //the real measurement plus noise
        // //correct
        // x_est = x_temp_est + K * (z_measured - x_temp_est); 
        // P = (1- K) * P_temp;
        // //we have our new system
        
		// if(isnan(x_est) || isinf(x_est)){
		// 	x_est = x_est_last;
		// }
        // printf("Ideal    position: %6.3f \n",z_real);
        // printf("Mesaured position: %6.3f [diff:%.3f]\n",z_measured,fabs(z_real-z_measured));
        // printf("Kalman   position: %6.3f [diff:%.3f]\n",x_est,fabs(z_real - x_est));
        
        // //update our last's
        // P_last = P;
        // x_est_last = x_est;

		x_est = position;

		error = 100 - x_est;
		error_sum += error;
		steer = KP * error + KI * error_sum + KD * (error - prev_error);
		steer = max(steer, -1.f);
		steer = min(steer, 1.f);
		prev_error = error;
	}
	
	printf("steer : %f\n", steer);
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
	hconcat(result, detectImg, result);
	
	imshow("result", result);
	outputVideo << result;
	if(waitKey(10) == 0) {
		return;
	}
	return;
}
#endif