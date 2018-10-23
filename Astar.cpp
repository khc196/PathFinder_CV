#include "Astar.h"

Astar::Astar(int width, int height) {
    if(width > MAXS || height > MAXS){
        printf("Size must be smaller than (200,200)\n");
        exit(1);
    }
    this->width = width;
    this->height = height;
    q = NULL;
}
void Astar::add_openlist(VERTEX v) {
    VERTEX temp;
    int cnt  = 0;
    int i, j, w, gx;

    // check the vertexs that are located on the nearest points.
    for( i = v.c-1 ;i <= v.c+1 ; i ++)
    {
        // if the point is off the map
        if( i < 0 || i >= width) continue;

        for( j = v.r ; j >=  v.r-1 ; j--)
        {
            // if the point is off the map or same with current point,
            // the points that can't move.
            if( j < 0 || (i == v.c && j == v.r) || visit[i][j] <= DONTMOVE) {
                continue;
            }
            // calculate the weight about the point
            w = calc_heuristic( v, i, j, &gx);
            //printf("(%d, %d) : %d\n", v.c, v.r, w);

            // if the weight that have calculated now is lower than
            // original weight.
            if( w < g[i][j] || g[i][j] == INF)
            {
                g[i][j]  = w;
                pre[i][j] = (v.c * width) + v.r;

                // if this point is the same with ending point.
                if( e.c == i && e.c == j)
                {
                    q = NULL;
                    return;
                }
            }
            temp.c = i;
            temp.r = j;
            temp.g = gx;
            // enqueue this point.
            //printf("enque : (%d, %d)\n", temp.c, temp.r);
            enqueue( temp);
            img2.data[j * img2.step + i*3 + 0] = 125;
            img2.data[j * img2.step + i*3 + 1] = 125;
            img2.data[j * img2.step + i*3 + 2] = 0;
        }
    }
}
int Astar::calc_heuristic(VERTEX v, int c, int r, int *gx) {
    int result;
    // calculate h(x) value.
    result = ((abs(e.c - c) + abs(e.r - r)) * 10);
    // get g(x) value of previous vertex.
    *gx = v.g;
    // examine whether this point is located on the diagonal.
    // increase the count of moving
    if( abs(v.c - c) == abs(v.r - r))
    {
        *gx = *gx + 14;  
    }
    else
    {
        *gx = *gx + 10;
    }
    return result + *gx;
}
void Astar::enqueue(VERTEX v) {
    QUEUE *f = q;
    QUEUE *newq = (QUEUE*)malloc(sizeof(QUEUE));
    VERTEX temp;
    int cnt  = 0;
    int key;

    newq->next = NULL;
    newq->v  = v;

    if( f == NULL)
    {
        q = newq;
        return;
    }

    // with insertion-sort, begin sorting process.
    while( f->next != NULL)
    {
        key = g[v.c][v.r];

        if( key < g[f->v.c][f->v.r])
        {
            temp = f->v;
            f->v = v;
            v  = temp;
        }

        f = f->next;
    }
    newq->v = v;
    f->next = newq;
}
VERTEX Astar::dequeue(void) {
    QUEUE *f = q;
    VERTEX v = {0,0,0};

    if( f != NULL)
    {
        q = f->next;

        v.c = f->v.c;
        v.r = f->v.r;
        v.g = f->v.g;

        free(f);

        return v;
    }
    return v;
}
int Astar::empty_queue(void) {
    return q == NULL;
}
void Astar::load_map(Mat src) {
    img = Mat(height, width, CV_8UC1);
    img2 = Mat(height, width, CV_8UC3);
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++){
            if(src.data[i * src.step + j] > 0){
                map[j][i] = WALL;
                img.data[i * img.step + j] = 255;
            }
            else{
                map[j][i] = 0;
                img.data[i * img.step + j] = 0;
            }
        }
    }
    cvtColor(img, img, CV_GRAY2BGR);
}
int Astar::find_path(Point sp, Point ep) {
    printf("find path from (%d, %d) to (%d, %d)\n", sp.x, sp.y, ep.x, ep.y);
    VERTEX v;
    s.c = sp.x;
    s.r = sp.y;
    e.c = ep.x;
    e.r = ep.y;
    if(map[ep.x][ep.y] < 0) {
        return 100000;
    }
    
    memset(visit, 0, sizeof(int)*width*height);
    memcpy(visit, map, sizeof(map));
    memset(g, 0, sizeof(int)*width*height);
    memset(pre, 0, sizeof(int)*width*height);
    q = NULL;
    g[s.c][s.r]  = 0; // init weight on the starting point.
    pre[s.c][s.r] = UNDEF; // the starting point don't have previous root.
    s.g    = 0; // it means that the number of moving.
    v = s; // make the starting point as current point.
    // add adjacency vertexs to the open list.
    add_openlist(v);
    img.copyTo(img2(Rect(0, 0, width, height)));
    img2.data[e.r * img.step + e.c*3 + 0] = 0;
    img2.data[e.r * img.step + e.c*3 + 1] = 0;
    img2.data[e.r * img.step + e.c*3 + 2] = 255;
    
    while(!empty_queue())
    {
        
        // add current vertex to the closed list.
        visit[v.c][v.r] = CLOSED;
        img2.data[v.r * img2.step + v.c*3 + 0] = 255;
        img2.data[v.r * img2.step + v.c*3 + 1] = 0;
        img2.data[v.r * img2.step + v.c*3 + 2] = 0;
        //imshow("img", img2);
        //waitKey(0);
        //printf("%d %d, %d\n", v.c, v.r, visit[v.c][v.r]);
        // update current vertex
        v = dequeue();
        if(visit[v.c][v.r] == WALL) {
            continue;
        }
        if(e.c == v.c && e.r == v.r) {
            break;
        }
        // add adjacency vertexs to the open list.
        add_openlist(v);
        // resize(img2, img2, Size(200, 200));
        // imshow("img", img2);
        // waitKey(0);
        // resize(img2, img2, Size(20, 20));
    }

    if (v.c == e.c && v.r == e.r) {
        printf("Done! - cost = %d\n", g[v.c][v.r]);
        return g[v.c][v.r];
    }
    else {
        printf("Failed!\n");
        return 100000;
    }
}