
// #pragma once

// #include <math.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <time.h>
// #include <cassert>
// #include <iostream>
// #include <sstream>
// #include <string>
// #include <vector>

// #ifdef _WIN32
// #define WINDOWS_LEAN_AND_MEAN
// #define NOMINMAX
// #include <windows.h>
// #endif

// #if defined(__APPLE__) || defined(MACOSX)
// #include <GLUT/glut.h>
// #else
// #include <GL/glut.h>
// #endif

// #include "gui/model/model.h"
// #include "gui/camera/camera.h" 

// using namespace std;
// namespace DynaMap{
// //*****************************GLUT call-back functions**********************
// // GLUT calls this function when the windows is resized
// void reshape(int w, int h, int& width, int& height) {
//   // Save new screen dimensions
//   width = (GLdouble)w;
//   height = (GLdouble)h;

//   // Ensuring our windows is a square
//   if (height == 0) {
//     height = 1;
//   };

//   if (width > height) {
//     width = height;
//   } else {
//     height = width;
//   };

//   // Tell OpenGL to use the whole window for drawing
//   glViewport(0, 0, (GLsizei)width, (GLsizei)height);

//   // Tell GLUT to call the redrawing function
//   glutPostRedisplay();
// }

// // GLUT redraws every given milliseconds
// void timer(int millisec) {
//   glutTimerFunc(millisec, timer, millisec);
//   glutPostRedisplay();
// }

// // GLUT calls this function when a key is pressed
// void keyboard(GLuint* vbo, gui::axis& mainCamera, unsigned char key, int x, int y) {
//   switch (key) {
//       // Quit when ESC or 'q' is pressed
//     case 27:
//     case 'q':
//     case 'Q':
//       exit(0);
//       break;
//     // Add a point when '+' is pressed
//     case '+':
//         n += 1;
//         break;

//     // Remove a point when '+' is pressed
//     case '-':
//         n -= 1;
//         break;

//     // Run/Stop the Marching Cubes algorithm when 'm' is pressed
//     case 'm':
//     case 'M':
//       setCUDA();
//         break;

//     // Change the surface to render when 'f' is pressed
//     case 'f':
//     case 'F':
//       changeFunction();
//         break;

//     // Show/hide the grid when 'g' is pressed
//     case 'g':
//     case 'G':
//     //   grid = 1-grid;
//       break;

//     // Show/hide the points when 'g' is pressed
//     case 'p':
//     case 'P':
//     //   pts = 1-pts;
//       break;

//     // Show/hide the surface when 's' is pressed
//     case 'x':
//     case 'X':
//     //   geom = 1-geom;
//       break;
//     case 'w':
//       mainCamera.y += 0.1f;
//       break;
//     case 's':
//       mainCamera.y -= 0.1f;
//       break;
//     case 'd':
//       mainCamera.x += 0.1f;
//       break;
//     case 'a':
//       mainCamera.x -= 0.1f;
//       break;

// 	  default:
// 	      ;
//   }
//   deleteVBOs(vbo);
//   createVBOs(vbo);
// }

// // GLUT calls this function when a mouse button is pressed
// void mouse(int button, int state, int x, int y) {
//   mouseX = x;
//   mouseY = y;

//   if (button == GLUT_LEFT_BUTTON) {
//     if (state == GLUT_DOWN) {
//       mouseLeftDown = 1;
//     } else if (state == GLUT_UP)
//       mouseLeftDown = 0;
//   }

//   else if (button == GLUT_RIGHT_BUTTON) {
//     if (state == GLUT_DOWN) {
//       mouseRightDown = 1;
//     } else if (state == GLUT_UP)
//       mouseRightDown = 0;
//   }

//   else if (button == GLUT_MIDDLE_BUTTON) {
//     if (state == GLUT_DOWN) {
//       mouseMiddleDown = 1;
//     } else if (state == GLUT_UP)
//       mouseMiddleDown = 0;
//   }
// }

// // GLUT calls this function when mouse is moved while a button is held down
// void motion(gui::axis& mainCamera, int x, int y, int mouseX, int mouseY) {
//   if (mouseLeftDown) {
//     mainCamera.pitch += (x - mouseX);
//     mainCamera.roll += (y - mouseY);
//     mouseX = x;
//     mouseY = y;
//   }
//   if (mouseRightDown) {
//     mainCamera.z -= (y - mouseY) * 0.2f;
//     mouseY = y;
//   }
// }

// } // namespace DynaMap