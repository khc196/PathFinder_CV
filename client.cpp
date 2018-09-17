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
    int shmid, shmid2;
    key_t skey, skey2;
    void *shared_memory, *shared_memory2;
    skey = 5678;
    skey2 = 1234;
    
    shmid = shmget((key_t) skey, sizeof(unsigned char) * 640 * 480 * 3, 0777);
    if (shmid == -1) {
		perror("shmget failed :");
        exit(1);
	}
    shared_memory = shmat(shmid, (void *) 0, 0);
	if (!shared_memory) {
		perror("shmat failed");
		exit(1);
	}

    shmid2 = shmget((key_t) skey2, sizeof(int), 0777);
    if (shmid == -1) {
		perror("shmget failed :");
        exit(1);
	}
    shared_memory2 = shmat(shmid2, (void *) 0, 0);
	if (!shared_memory) {
		perror("shmat failed");
		exit(1);
	}
    int* torcs_steer = (int *) shared_memory2;
    Path_Finder* pf = new Path_Finder();
    pf->init();
    while(true){
        unsigned char* img = (unsigned char*)shared_memory;
        Mat origin_img(480, 640, CV_8UC3, img);
        memcpy(img, origin_img.data, sizeof(unsigned char) * 640 * 480 * 3);
        Mat flipped_img;
        flip(origin_img, flipped_img, 0);
        *torcs_steer = pf->operate(flipped_img);
    }
    delete pf;
}

