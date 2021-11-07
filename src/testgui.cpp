///////////////////////////////////////////////////////////////////////////////
// MARCHING CUBES
// //
///////////////////////////////////////////////////////////////////////////////
// CS179 - SPRING 2014
// Final project
// Victor Ceballos Inza

// This projects consists in an implementation of the marching cubes algorithm
// in three different dimensions, using the GPU to accelerate the process.

// This file contains the rendering functions.

///////////////////////////////////////////////////////////////////////////////
// Includes, system
// //
///////////////////////////////////////////////////////////////////////////////
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cassert>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

#include "gui/openGL/openGL.h"

int argc;
char **argv;

void runGUI(void);
void gui_callback(void) {
  runGUI();
}

int main(int _argc, char** _argv) {
argc = _argc;
  argv = _argv;
   // Launch a thread
  std::thread guiThread(gui_callback);
  // Join the thread with the main thread
  guiThread.join();
  return 0;
}

void runGUI(void) {
  // Print usage
  printf(
      "This is an implementation of the marching cubes algorithm in CUDA.\n"
      "\nAlgorithm options:\n"
      "\t'm'         stops/starts the algorithm.\n"
      "\t'f'         changes the function of the surface to render.\n"
      "\t'1','2','3' changes the dimension of the algorithm.\n"
      "\t'+','-'     changes the number of points in the algorithm.\n"
      "\nRendering options:\n"
      "\t'g' shows/hides the grid.\n"
      "\t'p' shows/hides the points.\n"
      "\t's' shows/hides the resulting surface.\n"
      "\nViewing options:\n"
      "\t'ESC' and 'q' quits the program.\n"
      "\t'Left click' rotates the figure.\n"
      "\t'Right click' zooms in/out.\n\n");

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
  
  // Initialize the data.
  createVBOs(vbo);
  
  // Set up GLUT call-backs.
  glutDisplayFunc(display);
  glutTimerFunc(33, timer, 33);  // redraw only every given millisec
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  
  // // Start the GLUT main loop
  glutMainLoop();
}