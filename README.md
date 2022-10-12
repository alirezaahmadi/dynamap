# DynaMap
## Project : Registration Techniques for Non-rigid Objects (C++, CUDA Implementation)
by: Alireza Ahamdi, 

The DynaMap is a purely C++/CUDA mapping framework which I developed within 6 months during my master Thesis in Institute of Photogrametry and Robotics of University of Bonn (IPB). It overall includes two major parts (rigid and Non-rigid Mapping) with so many stand alone accelerated (pure CUDA based) libraries which I implemented from scratch like (dual-quaternions, Meshing, Rasterising, raycasting, Mesh Blending, different Data structures, KDtee and KNN and more).
The rigid section is tested with famous RGBD datasets like TUM, IPB captures achieving state of the art performances (both in speed and accuracy) and the non-rigid part due to complexity could be tested only on synthetic.

I will try to put more explanation of the code now and later in this repo so any contributions is really appreciate!

so feel free to reach-out and enjoy coding.. 

## Youtube Demo On rigid Mapping

<div align="center">

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/cDG6tOieziQ/0.jpg)](https://www.youtube.com/watch?v=cDG6tOieziQ)

</div>

<div align="center">
	
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Nr2SV6QNiAs/0.jpg)](https://www.youtube.com/watch?v=Nr2SV6QNiAs)
	
</div>

 ----
 
## General problem explanation:

In general, the problem of non-rigid registration is about matching two different
scans of a dynamic object taken at two different points in time. These scans
can undergo both rigid motions and non-rigid deformations. Since new parts
of the model may come into view and other parts get occluded in between two
scans, the region of overlap is a subset of both scans. In the most general
setting, no prior template shape is given and no markers or explicit feature
point correspondences are available. So, this case is a partial matching problem
which takes into account the assumption that consequent scans undergo small
deformations while having a significant amount of overlapping area [28]. The
problem which this thesis is addressing is about mapping deforming objects and
localizing camera in the environment at the same time.

<div align="center">
	<img src="/doc/nr.png" alt="cadf" width="400" title="cadf"/>
	<img src="/doc/warpfield.png" alt="robotoutside" width="400" title="robotoutside"/>
</div>

In this thesis, we contribute to the subject of the SLAM problem, where a
robotic agent navigates in the environment and constructs a dense 3D map of
the surrounding static and dynamic objects. In this section, we describe the
main contributions of this thesis as follows:
• The first contribution of this thesis is an approach for constructing a dense
representation of the environment while accurately tracking camera pose.
The proposed technique is based on KinectF usion [25] and voxel hashing
technique [40]. Our approach uses geometric and photometric information
of the scene to estimate the camera pose precisely. The performance of our
approach is evaluated via different data-sets of static scenes from TUM
and NUIM collections, showing the robustness of our approach in handling harsh movements and dealing with outliers. The proposed method
is explained in details in Chap. 3.
• The second contribution of this work is an approach that enables robotic
agents to map objects deformations. The proposed method can model
deformations of a non-rigid object without using any pre-build model or
any explicit template. Our method is inspired from deformation graph
idea [49] and DynamicF usion [37], which enables us to map non-rigid
scenes.
In sum, this thesis presents two contributions, that address different issues
in the context of dense 3D mapping. All the methods are designed to work
online on real-world data. Also, all algorithms are developed using C++ and
CUDA libraries to reach real-time performance.

<div align="center">
	<img src="/doc/warpfield_data.png" alt="oldrobot" width="400" title="oldrobot"/>
</div>

## Installation
```

sudo apt install nvidia-cuda-toolkit
nvcc -V

sudo apt install build-essential

sudo apt install gcc-6 g++-6
make sure gcc6 is selected
sudo update-alternatives --config gcc
or
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 60
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60
```

### Eigen
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.zip 

- unzipthe file,
- navigate into eign.x.x
- run `mkdir build`
- `cd build`
- `cmake ..`
- `sudo make install`

### How to install GLFW
```
sudo apt-get update 
sudo apt-get install libglfw3 libglfw3-dev libglew-dev libglm-dev 
sudo apt-get install libsoil-dev libglm-dev libassimp-dev libglew-dev libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev
```


## Deployment

## **Contribution**

Any contribution is highly appreciated 
dont hesitate to contact me! 
alireza.ahmadi@uni-bonn.de


---

<div align="center">
  
[![paypal](https://pics.paypal.com/00/s/NGRhNWNlODUtMzZlOS00MjJhLTg2NDEtMzNiNzczMTZkMDU4/file.PNG)](https://www.paypal.com/donate/?hosted_button_id=23TQAZ9MSLAUU)

</div>

---


**Start date: 5.6.2019**
**Status**
- Implemented basic stuff 
- basic Deformation graph, 
- RGBD to point-cloud,
- Point Cloud Downsampling
- Basic Solver library to contain implementations of non linear least-squares.
- Geometry namespace containing (libraries related to Mesh, voxelGrid, basic geometric stuff)
- Util namespace containing (handling IO, string and binary related files and basic math operations)
- ROS node and related function (subscribers, publishers, TF related operator)
- KNN search with and without FLANN and KDtree
- Point cloud to mesh using PCL,
- Visualizers using PCL, 
- ICP variants (least squares and SVD methods)
- Read cuda basic stuff and added to project (along with simple kernels)
- Evaluation functions (MSE, overall error)
- Sparse feature detection and matching functions (ORB,SIFT,SURF)
- A simple and not complete implementation of General RANSAC method 
- Local density filter for downsampling purposes of point cloud 

**repository address is**
https://gitlab.igg.uni-bonn.de/Alireza/non-rigid-mapping 

**date: 1 June - 11 June** 
- rough implementation of data structure of deformation graph
- learning cuda 
- writing libraries to load and save mesh .obj and .ply files 
- writing library to load RGBD datasets  

**date 12 June - 21 June**
- implementation of ICP based on point clouds and SVD solver for tracking camera pose
- eigen library based
- using nearest neighbor approach for picking correspondences  both with and without kd-tree
- along with down sampling with local density check and random sampling
- building an easy to use library of pcl visualizer 

**date: 22 June - 24 June**
- preparation of virtual depth and rgb image (taken from TSDF ) to be used as target image for registration process. ( frame to model registration) 

**date: 25 June - 26 June**
- implementation of bilateral filter for depth images based on gaussian kernel 



## Citation
---

```bash

@article{ahmadi2021registration,
  title={Registration Techniques for Deformable Objects},
  author={Ahmadi, Alireza, Cyrill Stachniss},
  journal={arXiv preprint arXiv:2111.04053},
  year={2021}
}

```


