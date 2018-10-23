#ifndef ASTAR_H
#define ASTAR_H

#include <malloc.h>
#include <cv.h>
#include <unistd.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

#define DONTMOVE -1
#define WALL  -2
#define CLOSED  -3
#define UNDEF  -1
#define INF   0
#define MAXS 200

typedef struct vertex{
 int c;
 int r;
 int g;
}VERTEX;

typedef struct queue{
 VERTEX v;
 struct queue *next;
}QUEUE;

class Astar {
private:
    void add_openlist(VERTEX);
    int calc_heuristic(VERTEX, int, int, int*);
    void enqueue(VERTEX);
    VERTEX dequeue(void);
    int empty_queue(void);
    int g[MAXS][MAXS];
    int map[MAXS][MAXS];
    int visit[MAXS][MAXS];
    int pre[MAXS][MAXS];
    int width;
    int height;
    QUEUE* q;
    VERTEX s, e;
    Mat img, img2;
public:
    Astar(int, int);
    void load_map(Mat);
    int find_path(Point, Point);
    int* get_visit();
};

#endif