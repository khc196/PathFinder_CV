#include "Path_Finder.hpp"
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <string>  
#include <math.h>
#include <queue>
#include <time.h>
using namespace cv;
using namespace std;

const Size imageSize = Size(640, 480);

int main(int argc, char* argv) 
{ 
    VideoCapture capture;
    capture.open(-1);
    
    if (!capture.isOpened()){
        cerr << "Camera error" << endl;
        return -1;
    }
    Mat map1, map2;
    Mat cameraMatrix = Mat::eye(3, 3, CV_64FC1);
    Mat distCoeffs = Mat::zeros(1, 5, CV_64FC1);
    
	FileStorage fs;
	fs.open("camcalib.xml", FileStorage::READ);
	
	if(!fs.isOpened()){
		cerr << "Failed to opend" << endl;
		return -1;
	}

	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;

	fs.release();
    
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);
    Path_Finder* pf = new Path_Finder();
    fp->init();
    FILE* fp = fopen("/home/nvidia/Desktop/p2", "w");
    fprintf(fp, "%d", 0);
    fclose(fp);
    //queue<Mat> q;
    time_t init_time, current_time;
    while(true){
        Mat origin_img, cali_img;
        capture >> origin_img;
	    cali_img = origin_img.clone();
        remap(origin_img, cali_img, map1, map2, CV_INTER_LINEAR);
	    pf->operate(cali_img);
        for(int i = 0; i < pf->direction_vec.size(); i++){
            steer = q.front();
            q.pop();
            FILE* fp = fopen("/home/nvidia/Desktop/p2", "w");
            fprintf(fp, "%f", steer);
	        fclose(fp);
            printf("steer : %f\n", steer);
        }
	    	
    }
    delete pf;
}

