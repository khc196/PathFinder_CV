#include "Path_Finder.hpp"
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <string>  
#include <time.h>

#include <sys/shm.h>
#include <sys/sem.h>
#include <sys/ipc.h>
#include <unistd.h>

using namespace cv;
using namespace std;

int main(int argc, char* argv) 
{ 
    int size_of_shared_memory = sizeof(unsigned char*) * 640 * 480 * 3;
    int shmid;
    key_t skey;
    void *shared_memory;
    skey = 5678;
    shmid = shmget((key_t) skey, size_of_shared_memory + sizeof(int) * 5, 0777);
    if (shmid == -1) {
		perror("shmget failed :");
        exit(1);
	}
    shared_memory = shmat(shmid, (void *) 0, 0);
	if (!shared_memory) {
		perror("shmat failed");
		exit(1);
	}
    Path_Finder* pf = new Path_Finder();
    pf->init();
    int* torcs_lock = (int*)(shared_memory + size_of_shared_memory + sizeof(int));
    struct timeval start_point, end_point;
    double operating_time;
    gettimeofday(&start_point, NULL);
    *torcs_lock = 1;
    while(true){
        if(*torcs_lock == 1) {
            gettimeofday(&end_point, NULL);
            operating_time = (double)(end_point.tv_sec)+(double)(end_point.tv_usec)/1000000.0-(double)(start_point.tv_sec)-(double)(start_point.tv_usec)/1000000.0;
            printf("operating time : %f\n\n", operating_time);
            gettimeofday(&start_point, NULL);
            unsigned char* img = (unsigned char*)shared_memory;
            float* torcs_steer = (float*)(shared_memory + size_of_shared_memory);
            int* torcs_speed = (int*)(shared_memory + size_of_shared_memory + sizeof(int)*2);
            Mat origin_img(480, 640, CV_8UC3, img);
            memcpy(img, origin_img.data, size_of_shared_memory);
            Mat flipped_img;
            flip(origin_img, flipped_img, 0);
            pf->operate(flipped_img);
            *torcs_steer = pf->steer;
            *torcs_speed = 80 / 3.6;
            *torcs_lock = 0;
        }
    }
    delete pf;
}

