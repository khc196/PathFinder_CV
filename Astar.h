#ifndef ASTAR_H
#define ASTAR_H

#include <malloc.h>
#include <cv.h>

using namespace std;
using namespace cv;

#define DONTMOVE -1
#define WALL  -2
#define CLOSED  -3
#define UNDEF  -1
#define INF   0
#define MAX 200

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
    int g[MAX][MAX];
    int map[MAX][MAX];
    int visit[MAX][MAX];
    int pre[MAX][MAX];
    int width;
    int height;
    QUEUE* q;
    VERTEX s, e;
public:
    Astar(int, int);
    void load_map(Mat);
    int find_path(Point, Point);
    int* get_visit();
};

#endif