/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/


#include "gui/openGL/openGL.h"

using namespace DynaMap;
using namespace gui;

// settings
#define SCR_WIDTH = 800;
#define SCR_HEIGHT = 600;

int main(int argc, char **argv) {
  assert(argc == 2);					// check # of arguments
  // Print usage
  // printf(
  //     "This is an implementation of the marching cubes algorithm in CUDA.\n"
  //     "\nAlgorithm options:\n"
  //     "\t'm'         stops/starts the algorithm.\n"
  //     "\t'f'         changes the function of the surface to render.\n"
  //     "\t'1','2','3' changes the dimension of the algorithm.\n"
  //     "\t'+','-'     changes the number of points in the algorithm.\n"
  //     "\nRendering options:\n"
  //     "\t'g' shows/hides the grid.\n"
  //     "\t'p' shows/hides the points.\n"
  //     "\t's' shows/hides the resulting surface.\n"
  //     "\nViewing options:\n"
  //     "\t'ESC' and 'q' quits the program.\n"
  //     "\t'Left click' rotates the figure.\n"
  //     "\t'Right click' zooms in/out.\n\n");

  // Initialize GLUT
  glutInit(&argc, argv);

  // Display mode
  glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH | GLUT_STENCIL);

  // Initialize the window settings.
  glutInitWindowSize(400,400);
  glutInitWindowPosition(800, 200);
  glutCreateWindow("Display");

  // Initialize the scene.
  initGL();
  // Set background (or clearing) color.
   glClearColor(1.0, 1.0, 1.0, 0.0); 
  
  
  // // load the model
  // model_object.loadFile(argv);

  // // Initialize the data.
  createVBOs(vbo);
  // Create a Vector Buffer Object that will store the vertices on video memory

  // 4 triangles to be rendered
	

  // init Axes
  MainCamera.z = -1.0f;
  // PredictedAxis.init();
  // GroundTruthAxis.init(5.0f);

  // cudaMemcpy(glPoints, mapBuilder.targetPCL->points, sizeof(float3) * Xtion.rows * Xtion.cols, cudaMemcpyDeviceToHost);

  // Set up GLUT call-backs.
  glutDisplayFunc(display);
  glutTimerFunc(33, timer, 33);  // redraw only every given millisec
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  
  // // Start the GLUT main loop
  glutMainLoop();
  return 0;
}
