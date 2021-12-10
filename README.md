# DynaMap
## Project : Registration Techniques for Non-rigid Objects
by: Alireza Ahamdi, 

### Youtube Demo On rigid Mapping

<div align="center">
	
[![YouTube Demo](https://i.ytimg.com/an_webp/Nr2SV6QNiAs/mqdefault_6s.webp?du=3000&sqp=CKLuvI0G&rs=AOn4CLBC9fBXmokbUYNog4GzBFnTqiv2Mg)](https://youtu.be/Nr2SV6QNiAs)
[![YouTube Demo](https://i.ytimg.com/an_webp/cDG6tOieziQ/mqdefault_6s.webp?du=3000&sqp=CKzhvI0G&rs=AOn4CLCNhiqWK1_6RaUk-g5Dw3olds6eZA)](https://youtu.be/cDG6tOieziQ)
	
</div>

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


install dependencies:

sudo apt install build-essential

sudo apt install gcc-6 g++-6
make sure gcc6 is selected
sudo update-alternatives --config gcc
or
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 60
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60

## Deployment

## **Contribution**

Any contribution is highly appreciated 
dont hesitate to contact me! 
alireza.ahmadi@uni-bonn.de

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


