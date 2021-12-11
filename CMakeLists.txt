cmake_minimum_required(VERSION 3.0.0)
project(dynamap VERSION 0.1.0)

add_compile_options(-std=c++11)

## Debugging things vs Release Optimizations
##   Set flags for each mode
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_FORCE_INLINES ")
# set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3")
##   Default to release if not specified

# cmake .. -DCMAKE_BUILD_TYPE=Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()
message("Build type: ${CMAKE_BUILD_TYPE}")
##   Type dependent definitions
if(${CMAKE_BUILD_TYPE} MATCHES Debug)
  add_definitions(-DDEBUG_MODE)
endif()


# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system signals)




find_package(catkin REQUIRED)
find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 3.3.7 REQUIRED)
find_package(catkin COMPONENTS 
     robovision 
     glow
     projection_example
     message_generation
     genmsg
     roscpp
     sensor_msgs
     std_msgs
     pcl_ros
     cv_bridge
     message_filters
)


catkin_package(
  CATKIN_DEPENDS
    glow
    robovision
    projection_example
    std_msgs
    message_runtime
    sensor_msgs
  DEPENDS
    Boost
)

# // to fix conflict of PCL and cuda
get_directory_property(dir_defs DIRECTORY ${CMAKE_SOURCE_DIR} COMPILE_DEFINITIONS)
set(vtk_flags)
foreach(it ${dir_defs})
    if(it MATCHES "vtk*")
    list(APPEND vtk_flags ${it})
    endif()
endforeach()

foreach(d ${vtk_flags})
    remove_definitions(-D${d})
endforeach()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}

    ${projection_example_INCLUDE_DIRS}

    include/dataStructure
    include/dataStructure/heap
    include/dataStructure/kdtree
    include/dataStructure/hashTable

    include/image
    include/image/sensor
    include/image/kernel2D
    include/image/conv2D
    include/image/pyramid

    include/builder

    include/solver
    # include/solver/icpSVD
    include/solver/gnSolver
    include/solver/nonRigidICP

    include/math

    include/blender
   
    include/geometry
    include/geometry/tsdf
    include/geometry/mesh

    include/geometry/defGraph
    include/geometry/raytracing
    include/geometry/marchingCubes
    include/geometry/mesh/meshExtractor
    
    include/utils
)

## Declare a cpp library
add_library(
    ${PROJECT_NAME}_core
    include/utils/utils_String.cpp
    include/utils/utils_Dataset.cpp
    include/geometry/mesh/mesh_IO.cpp
    # include/nodehandler/nodehandler.cpp
)

target_link_libraries(
    ${PROJECT_NAME}_core 
    ${catkin_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    ${OpenCV_LIBS} 
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES} 
    Eigen3::Eigen
)

add_subdirectory(include/gui)

set(CUDA_SEPARABLE_COMPILATION ON)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --ptxas-options=-v")
SET(CUDA_NVCC_FLAGS "-arch=sm_50" CACHE STRING "nvcc flags" FORCE)
SET(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
# SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;-std=c++11 -gencode arch=compute_50,code=sm_50)

SET(LIB_TYPE STATIC)
CUDA_ADD_LIBRARY(${PROJECT_NAME}_cuda 
    ${LIB_TYPE}

    # include/geometry/knn/knn.cu
    include/geometry/tsdf/tsdf.cu
    include/geometry/mesh/mesh.cu
    include/geometry/mesh/meshSTD.cu
    include/geometry/defGraph/defGraph.cu
    include/geometry/rayTracing/rayTracing.cu
    include/geometry/mesh/meshExtractor/meshExtractor.cu

    include/blender/dqBlender/dqBlender.cu

    include/geometry/marchingCubes/marchingCubes.cu

    include/image/conv2D/conv2D.cu
    include/image/pyramid/pyramid.cu
    include/image/sensor/rgbdImage.cu
    include/image/kernel2D/kernel2D.cu
    
    include/solver/nonRigidICP/nonRigidICP.cu
    include/solver/gnSolver/gnSolver.cu
    include/builder/builder.cu

    include/dataStructure/heap/heap.cu
    include/dataStructure/kdtree/kdtree.cu
    include/dataStructure/hashTable/hashTable.cu

    # include/utils/utils_Cuda.cu
)

target_link_libraries(${PROJECT_NAME}_cuda 
  ${PROJECT_NAME}_gui
  -lcublas
  -lcurand
  -lcusparse 
  -lcusolver
)


add_subdirectory(src)
add_subdirectory(src/test)
