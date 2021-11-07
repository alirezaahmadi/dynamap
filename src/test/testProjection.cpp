
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "glow/glbase.h"
#include "glow/glutil.h"
#include "glow/util/X11OffscreenContext.h"

#include "dqBlender/dqBlender.h"
#include "mesh/meshSTD.h"
#include "mesh/mesh_IO.h"


// #include "DepthProjector.h"
// #include "ObjReader.h"

// #include "gui/openGL/openGL.h"

// using namespace glow;

using namespace DynaMap;

int main(int argc, char **argv) {

  glow::X11OffscreenContext ctx;
  glow::inititializeGLEW();
 
  geometry::Mesh_IO inpMesh(argv[1]);
  if (argc >= 3 && atoi(argv[2]) == 1) inpMesh.Mesh2MeshSTD();

  geometry::MeshSTD *srcMesh;
  cudaMallocManaged(&srcMesh, sizeof(geometry::MeshSTD));
  srcMesh->Init(inpMesh.verticesNum, inpMesh.trianglesNum);

  cudaMemcpy(srcMesh->vertices, inpMesh.vertices, sizeof(geometry::Vertex) * srcMesh->verticesNum,
             cudaMemcpyHostToDevice);
  cudaMemcpy(srcMesh->triangles, inpMesh.triangles, sizeof(geometry::Polygon) * srcMesh->trianglesNum,
             cudaMemcpyHostToDevice);
  cudaDeviceSynchronize();

  uint32_t width = 640, height = 480;
  Eigen::Matrix4f P = glow::glPerspective(glow::radians(58.0f), 4.0 / 3.0, 0.1, 10.0f);  // World -> Image -> NDC
  Eigen::Matrix4f Tranform = glow::glRotateX(M_PI);
  Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
  // view(2,3) += -2.0f;
  view = Tranform * view;
  DepthProjector proj(width, height, P);

  std::vector<float3> vertices;
  for (size_t i = 0; i < srcMesh->trianglesNum; i++) {
    // **Attention*** -1 if because the vertex ids in mesh face begins from 1 
    vertices.push_back(srcMesh->vertices[srcMesh->triangles[i].vertexIndex.x - 1].position);    
    vertices.push_back(srcMesh->vertices[srcMesh->triangles[i].vertexIndex.y - 1].position);
    vertices.push_back(srcMesh->vertices[srcMesh->triangles[i].vertexIndex.z - 1].position);
  }
  std::cout << "vertices loaded..." << std::endl;

  proj.render(vertices, view);
  std::vector<glow::vec4> data(width * height);
  uint32_t count = 0;
  for (auto d : data) {
    count += (d.x > 0);
  }
  std::cout << "non-zeros = " << count << std::endl;
  proj.texture().download(data);
  proj.texture().save("../renders/test.ppm");

  cv::Mat fiIMG(480, 640, CV_32FC1);
  for (int i = 0; i < 480; i++) {
    for (int j = 0; j < 640; j++) {
      fiIMG.at<float>(i, j) = data[i * 640 + j].z;
    }
  }
  // fiIMG.convertTo(fiIMG, CV_32FC1, 5000);
  cv::imshow("depth",fiIMG);
  cv::imwrite( "../renders/depth.png", fiIMG );
  cv::waitKey(0);

  cudaDeviceSynchronize();
  cudaFree(srcMesh);

  return 0;
}
