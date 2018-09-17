#include "Path_Finder.hpp"
#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <string>  

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
    shmid = shmget((key_t) skey, size_of_shared_memory + sizeof(int) * 2, 0777);
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
    while(true){
        if(*torcs_lock == 1) {
            unsigned char* img = (unsigned char*)shared_memory;
            int* torcs_steer = (int*)(shared_memory + size_of_shared_memory);
            Mat origin_img(480, 640, CV_8UC3, img);
            memcpy(img, origin_img.data, sizeof(unsigned char) * 640 * 480 * 3);
            Mat flipped_img;
            flip(origin_img, flipped_img, 0);
            pf->operate(flipped_img);
            int size = pf->direction_vec.size();
            *torcs_steer = 0;
            for(int i = 0; i < size; i++){
                *torcs_steer += -pf->direction_vec.front();
                pf->direction_vec.pop();
            }
            *torcs_lock = 0;
        }
    }
    delete pf;
}

