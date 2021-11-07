#include <cuda.h>
#include <cuda_runtime.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <iostream>

#include "dataStructure/kdtree/kdtree.h"
#define MAX_DIM 	2
#define N 1000000
#define rand1() (rand() / (float)RAND_MAX)
#define rand_pt(v) { v.x[0] = rand1(); v.x[1] = rand1(); v.x[2] = rand1(); }

#define querySize 6

using namespace DynaMap;

int main(int argc, char **argv){

    kdTree kdtree;
    struct kdNode wp[] = {
        {{2, 3}}, {{5, 4}}, {{9, 6}}, {{4, 7}}, {{8, 1}}, {{7, 2}}
    };
    struct kdNode testNode = {{9, 2}};
    struct kdNode *root, *found, *million;
    float best_dist;
 
    root = kdtree.buildTree(wp, sizeof(wp) / sizeof(wp[1]), 0, 2);
 
    kdtree.visited = 0;
    found = 0;
    kdtree.findNearests(root, &testNode, 0, 2, &found, &best_dist);
 
    printf(">> WP tree\nsearching for (%g, %g)\n"
            "found (%g, %g) dist %g\nseen %d nodes\n\n",
            testNode.x[0], testNode.x[1],
            found->x[0], found->x[1], sqrt(best_dist), kdtree.visited);
 
    million =(struct kdNode*) calloc(N, sizeof(struct kdNode));
    srand(time(0));
    for (int i = 0; i < N; i++) rand_pt(million[i]);
 
    root = kdtree.buildTree(million, N, 0, 3);
    rand_pt(testNode);
 
    kdtree.visited = 0;
    found = 0;
    kdtree.findNearest(root, &testNode, 0, 3, &found, &best_dist);
 
    printf(">> Million tree\nsearching for (%g, %g, %g)\n"
            "found (%g, %g, %g) dist %g\nseen %d nodes\n",
            testNode.x[0], testNode.x[1], testNode.x[2],
            found->x[0], found->x[1], found->x[2],
            sqrt(best_dist), kdtree.visited);
 
    /* search many random points in million tree to see average behavior.
       tree size vs avg nodes visited:
       10          ~ 7
       100         ~ 16.5
       1000        ~ 25.5
       10000       ~ 32.8
       100000      ~ 38.3
       1000000     ~ 42.6
       10000000    ~ 46.7              */
    int sum = 0, test_runs = 100000;
    for (int i = 0; i < test_runs; i++) {
        found = 0;
        kdtree.visited = 0;
        rand_pt(testNode);
        kdtree.findNearests(root, &testNode, 0, 3, &found, &best_dist);
        sum += kdtree.visited;
    }
    printf("\n>> Million tree\n"
            "visited %d nodes for %d random findings (%f per lookup)\n",
            sum, test_runs, sum/(float)test_runs);
 
       free(million);
 
    return 0;
}
 

