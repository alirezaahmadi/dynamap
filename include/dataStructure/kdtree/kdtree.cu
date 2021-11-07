/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// based on https://rosettacode.org/wiki/K-d_tree
#include "kdtree.h"

namespace DynaMap{
__device__ __host__
kdTree::kdTree(){}
__device__ __host__
kdTree::~kdTree(){}
void kdTree::init(int querySize){
    visited = 0;
    cudaMallocManaged(&kdDistnaces, sizeof(float));
    cudaMallocManaged(&kdFound,     sizeof(struct kdNode));
    cudaMallocManaged(&kdRoot,      sizeof(struct kdNode) * NODE_NUM);
    cudaMallocManaged(&kdQuery,     sizeof(struct kdNode) * NODE_NUM);
    cudaMallocManaged(&VisitedNodes,sizeof(struct kdNode) * NODE_NUM);
    cudaDeviceSynchronize();
}
void kdTree::Free(void){
    cudaDeviceSynchronize();
    cudaFree(kdDistnaces);
    cudaFree(kdFound); 
    cudaFree(kdRoot); 
    cudaFree(kdQuery); 
    cudaFree(VisitedNodes); 
}
__device__ __host__
inline float kdTree::dist(struct kdNode *a, struct kdNode *b, int dim){
    float t, d = 0;
    while (dim--) {
        t = a->x[dim] - b->x[dim];
        d += t * t;
    }
    return d;
}
inline void kdTree::swap(struct kdNode *x, struct kdNode *y) {

    #if defined(__CUDA_ARCH__)
        struct kdNode *tmp;
        cudaMallocManaged(&tmp, sizeof(struct kdNode));

        cudaMemcpy(tmp, x, sizeof(struct kdNode), cudaMemcpyDeviceToDevice);
        cudaDeviceSynchronize();

        cudaMemcpy(x, y,   sizeof(struct kdNode), cudaMemcpyDeviceToDevice);
        cudaDeviceSynchronize();

        cudaMemcpy(y, tmp, sizeof(struct kdNode), cudaMemcpyDeviceToDevice);
        cudaDeviceSynchronize();
    #else
        float tmp[KDTREE_MAX_DIM];
        int id;
        memcpy(tmp,  x->x, sizeof(tmp));
        id = x->id;

        memcpy(x->x, y->x, sizeof(tmp));
        x->id= y->id;

        memcpy(y->x, tmp,  sizeof(tmp));
        y->id = id;
    #endif
}
struct kdNode* kdTree::findMedian(struct kdNode *start, struct kdNode *end, int idx){
    if (end <= start) return NULL;
    if (end == start + 1)
        return start;
 
    struct kdNode *p, *store, *md = start + (end - start) / 2;
    float pivot;
    while (1) {
        pivot = md->x[idx];
 
        swap(md, end - 1);
        for (store = p = start; p < end; p++) {
            if (p->x[idx] < pivot) {
                if (p != store)
                    swap(p, store);
                store++;
            }
        }
        swap(store, end - 1);
 
        /* median has duplicate values */
        if (store->x[idx] == md->x[idx])
            return md;
 
        if (store > md) end = store;
        else        start = store;
    }
}
struct kdNode* kdTree::buildTree(struct kdNode *t, int len, int i, int dim){
    struct kdNode *n;
 
    if (!len) return 0;
 
    if ((n = findMedian(t, t + len, i))) {
        i = (i + 1) % dim;
        n->left  = buildTree(t, n - t, i, dim);
        n->right = buildTree(n + 1, t + len - (n + 1), i, dim);
    }
    return n;
}
__device__ __host__
void kdTree::findNearest(struct kdNode *root, 
                          struct kdNode *nd, 
                          int i, 
                          int dim,
                          struct kdNode **best, 
                          float *best_dist){
    float d, dx, dx2;
 
    if (!root) return;
    d = dist(root, nd, dim);
    dx = root->x[i] - nd->x[i];
    dx2 = dx * dx;
 
    visited ++;
    // std::cout << "RootID: "<< root->id  << ", ndID: "<< nd->id  << ", d: " << d << std::endl;
    if (!*best || d < *best_dist) {
        *best_dist = d;
        *best = root;
    }
 
    /* if chance of exact match is high */
    if (!*best_dist) return;
 
    if (++i >= dim) i = 0;
 
    findNearest(dx > 0 ? root->left : root->right, nd, i, dim, best, best_dist);
    if (dx2 >= *best_dist) return;
    findNearest(dx > 0 ? root->right : root->left, nd, i, dim, best, best_dist);
}
__device__ __host__
void kdTree::findKNearest(struct kdNode *root, 
                          struct kdNode *nd, 
                          int i, 
                          int dim,
                          struct kdNode **best, 
                          float *best_dist,
                          struct kdNode *VisitedNodes){

    float d, dx, dx2;
 
    if (!root) return;
    d = dist(root, nd, dim);
    dx = root->x[i] - nd->x[i];
    dx2 = dx * dx;

    VisitedNodes[visited] = *root;
    VisitedNodes[visited].distance = d;
    visited ++;

    // std::cout << "RootID: "<< root->id  << ", ndID: "<< nd->id  << ", d: " << d << std::endl;

    if (!*best || d < *best_dist) {
        *best_dist = d;
        *best = root;
    }
    
    /* if chance of exact match is high */
    if (!*best_dist) return;
    
    if (++i >= dim) i = 0;
    
    findKNearest(dx > 0 ? root->left : root->right, nd, i, dim, best, best_dist, VisitedNodes);
    if (dx2 >= *best_dist) return;
    findKNearest(dx > 0 ? root->right : root->left, nd, i, dim, best, best_dist, VisitedNodes);
}  
__device__ __host__
void kdTree::findKNN(struct kdNode &targetNode){
    
    this->visited = 0;
    this->kdFound = 0;

    this->findKNearest(this->kdRoot, &targetNode, 0, 3, &this->kdFound, this->kdDistnaces, VisitedNodes);
    
    this->sortNodes(this->visited);
}
__device__ __host__
inline void kdTree::sortNodes(int visitedNum){

    for (int idx = 0; idx < visitedNum; idx++){
        // Shift values (and indexes) higher
        struct kdNode tmpNodes;
        int j = idx;
        // Store current distance and associated nIdx
        struct kdNode currNodes = VisitedNodes[j];
        while (j > 0 && VisitedNodes[j-1].distance > currNodes.distance) {

            tmpNodes  = VisitedNodes[j-1];

            VisitedNodes[j-1] = currNodes;

            VisitedNodes[j] = tmpNodes;
            
            --j;
        }
    }
}


}
