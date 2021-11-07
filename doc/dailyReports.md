 # Project : Different Techniques of Registration of non-Rogid Objects

 **Here, I am trying to put a short report of my progress through this project.**

### April
---

### May
---
* Read all the related papers (fusion series and Deformation graph related) and summarized them in attache files.
* Implemented basic stuff 
    * basic Deformation graph, 
    * RGBD to point-cloud,
    * Point Cloud Downsampling
    * Basic Solver library to contain implementations of non linear least-squares.
    * Geometry namespace containing (libraries related to Mesh, voxelGrid, basic geometric stuff)
    * Util namespace containing (handling IO, string and binary related files and basic math operations)
    * ROS node and related function (subscribers, publishers, TF related operator)
    * KNN search with and without FLANN and KDtree
    * Point cloud to mesh using PCL,    
    * Visualizers using PCL, 
    * ICP variants (least squares and SVD methods)
    * Read cuda basic stuff and added to project (along with simple kernels)
    * Evaluation functions (MSE, overall error)
    * Sparse feature detection and matching functions (ORB,SIFT,SURF)
    * A simple and not complete implementation of General RANSAC method 
    * Local density filter for downsampling purposes of point cloud 

### June
---
* date: 1 June - 11 June 
    - rough implementation of data structure of deformation graph
    - learning cuda 
    - writing libraries to load and save mesh .obj and .ply files 
    - writing library to load RGBD datasets  
* date 12 June - 21 June
    - implementation of ICP based on point clouds and SVD solver for tracking camera pose eigen library based
    - using nearest neighbor approach for picking correspondences  both with and without kd-tree
    - along with down sampling with local density check and random sampling
    - building an easy to use library of pcl visualizer 
* date: 22 June - 24 June
    - preparation of virtual depth and rgb image (taken from TSDF ) to be used as target image for registration process. ( frame to model registration) 
* date: 25 June - 26 June
    - implementation of bilateral filter for depth images based on gaussian kernel 

* date : 27 June - 2 July
getting normals from depth image and vertices 
implementation is of least squares (gauss-newton) method to be uesd as optimizer in SVD.
 
### July
---

### August
---

### September
---

### October 
---




