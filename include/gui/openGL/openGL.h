#pragma once

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <cassert>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>



#include "geometry/marchingCubes/marchingCubes.h"

#ifdef _WIN32
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#if defined(__APPLE__) || defined(MACOSX)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "gui/model/model.h"
#include "gui/camera/camera.h"

using namespace std;
namespace DynaMap{
///////////////////////////////////////////////////////////////////////////////
// Declarations
// //
///////////////////////////////////////////////////////////////////////////////

// Window settings
int width = 512;
int height = 512;

// Mouse controls
int mouseLeftDown = 0;
int mouseRightDown = 0;
int mouseMiddleDown = 0;
float mouseX = 0;
float mouseY = 0;
float axisAngleX = 0.0f;
float axisAngleY = 0.0f;
float axisDistance = 20.0f;

// Flags to toggle drawing
int grid = 0;
int pts =0;
int geom = 1;

int n = 6; 
int size = 0;

gui::axis MainCamera;
gui::axis PredictedAxis;
gui::axis GroundTruthAxis; 

// global variables
static int isWire = 1; // wireframe?
static int isFog = 0;  // fog?
int isOrtho = 1;		// projection mode ?

// VBOs
GLuint vbo[7];  // ID of VBO for vertex arrays - 0 is reserved
                // glGenBuffers() will return non-zero id if success
                // vbo[1] - grid
                // vbo[2] - points
                // vbo[3] - geometry
                // vbo[4] - Predicted axis pose
                // vbo[5] - Ground-truth axis pose
                // vbo[6] - Loaded Mesh


// OpenGL initialization
GLvoid initaxis();
GLvoid initLights();
GLvoid initMaterial();
GLvoid initTexture();
GLvoid initColors();
GLvoid initGL();

// Rendering functions
void drawOrigin();
void drawGrid();
void drawPoints();
void drawGeom();


// Call-backs
void display();
void reshape(int w, int h);
void timer(int millisec);
void keyboard(unsigned char key, int x, int y);
void mouse(int button, int state, int x, int y);
void motion(int x, int y);

///////////////////////////////////////////////////////////////////////////////
// Display callback
// //
///////////////////////////////////////////////////////////////////////////////
void display() {
  // Clear buffer
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  // reset transformation matrix
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // User Interaction matrices.
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(MainCamera.x, MainCamera.y, MainCamera.z);  // Zooming
  glRotatef(MainCamera.roll, 1.0, 0.0, 0.0);   // Rotations
  glRotatef(MainCamera.pitch, 0.0, 1.0, 0.0);
  glRotatef(MainCamera.yaw, 0.0, 0.0, 1.0);

  // Draw grid/points/geometry
  if (grid) {
    drawGrid();
  }
  if (pts) {
    drawPoints();
  }
  if (geom) {
    drawGeom();
  }

  // Mark position of the origin
  // glPushMatrix();
  drawOrigin();
  // glPopMatrix();

  // Mark position of the PredictedAxis 
  // glPushMatrix();
  // glTranslatef(PredictedAxis.x, PredictedAxis.y, PredictedAxis.z);  // Zooming
  // glRotatef(PredictedAxis.roll, 1.0, 0.0, 0.0);   // Rotations
  // glRotatef(PredictedAxis.pitch, 0.0, 1.0, 0.0);
  // glRotatef(PredictedAxis.yaw, 0.0, 0.0, 1.0);
  // PredictedAxis.draw();
  // glPopMatrix();

  // Mark position of the Ground-truth axis 
  // glPushMatrix();
  // glTranslatef(GroundTruthAxis.x, GroundTruthAxis.y, GroundTruthAxis.z);  // Zooming
  // glRotatef(GroundTruthAxis.roll, 1.0, 0.0, 0.0);   // Rotations
  // glRotatef(GroundTruthAxis.pitch, 0.0, 1.0, 0.0);
  // glRotatef(GroundTruthAxis.yaw, 0.0, 0.0, 1.0);
  // GroundTruthAxis.draw();
  // glPopMatrix();

  // Swap buffers.
  glutSwapBuffers();
  glutPostRedisplay();
}
///////////////////////////////////////////////////////////////////////////////
// Rendering functions //
///////////////////////////////////////////////////////////////////////////////
void drawOrigin() {
  // glColor3f(1.0, 1.0, 0.0);
  // glBegin(GL_POINTS);
  // glVertex4f(0.0, 0.0, 0.0, 1.0);
  // glEnd();
  // glColor3f(0.0, 0.0, 0.0);
  float axisLength = 0.1f;
  glLineWidth(5.0);
  // draw some lines
  // x aix
  glColor3f(1.0,0.0,0.0); // red x
  glBegin(GL_LINES);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(axisLength, 0.0f, 0.0f);
  glEnd();

  // y 
  glColor3f(0.0,1.0,0.0); // green y
  glBegin(GL_LINES);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0, axisLength, 0.0f);
  glEnd();

  // z 
  glColor3f(0.0,0.0,1.0); // blue z
  glBegin(GL_LINES);
  glVertex3f(0.0f, 0.0f, 0.0f);
  glVertex3f(0.0, 0.0f ,axisLength);

  glEnd();
  glLineWidth(1.0);
}

void drawGrid()
{
    // Bind VBOs with IDs.
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);

	// Enable vertex arrays.
    glEnableClientState(GL_VERTEX_ARRAY);

    // Specify pointer to vertex array.
    glVertexPointer(4, GL_FLOAT, 0, 0);

    // Set up the polygon mode.
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Set up the color.
	glColor3f(0.0,0.0,1.0);

	// Switch on the dimension
	int n = getNumPoints();
	int size = 0;

  // Get the right size of the grid to render
  size = (n-1)*(n-1)*(n-1)*16;
  // Render to screen
  glDrawArrays(GL_QUADS, 0, size);


	// Set color to default.
	glColor3f(0.0,0.0,0.0);

    // Disable vertex arrays.
    glDisableClientState(GL_VERTEX_ARRAY);

    // Release VBOs with ID 0 after use.
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void drawPoints()
{
    // Bind VBOs with IDs.
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);

	// Enable vertex arrays.
    glEnableClientState(GL_VERTEX_ARRAY);

    // Specify pointer to vertex array.
    glVertexPointer(4, GL_FLOAT, 0, 0);

    // Set up the polygon mode.
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// Switch on the dimension
	int n = getNumPoints();
	int size = 0;
  // Get the right number of points to render
  size = n*n*n;


    // Render to screen
	glDrawArrays(GL_POINTS, 0, size);

    // Disable vertex arrays.
    glDisableClientState(GL_VERTEX_ARRAY);

    // Release VBOs with ID 0 after use.
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void drawGeom()
{
    // Bind VBOs with IDs.
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);

	// Enable vertex arrays.
    glEnableClientState(GL_VERTEX_ARRAY);

    // Specify pointer to vertex array.
    glVertexPointer(4, GL_FLOAT, 0, 0);

	// Set up the color.
	glColor3f(1.0,0.0,0.0);

	// Switch on the dimension
	int n = getNumPoints();
	int size = 0;

		// Get the right dimension of the surface to render
		size = 2503;
    // std::cout << model_object.glModel.size() << std::endl;

		// Set up the polygon mode.
		// glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		// // Render to screen
		// glDrawArrays(GL_TRIANGLES, 0, size);
		// Set up the color.
		glColor3f(0.0,1.0,0.0);
		// Set up the polygon mode.
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		// Render to screen
		glDrawArrays(GL_TRIANGLES, 0, size);

    // Set color to default.
    glColor3f(0.0,0.0,0.0);

    // Disable vertex arrays.
    glDisableClientState(GL_VERTEX_ARRAY);

    // Release VBOs with ID 0 after use.
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

//*****************************GLUT call-back functions**********************
// GLUT calls this function when the windows is resized
void reshape(int w, int h) {
  // Save new screen dimensions
  width = (GLdouble)w;
  height = (GLdouble)h;

  // Ensuring our windows is a square
  if (height == 0) {
    height = 1;
  };

  if (width > height) {
    width = height;
  } else {
    height = width;
  };

  // Tell OpenGL to use the whole window for drawing
  glViewport(0, 0, (GLsizei)width, (GLsizei)height);

  // Tell GLUT to call the redrawing function
  glutPostRedisplay();
}

// GLUT redraws every given milliseconds
void timer(int millisec) {
  glutTimerFunc(millisec, timer, millisec);
  glutPostRedisplay();
}

// GLUT calls this function when a key is pressed
void keyboard(unsigned char key, int x, int y) {
  switch (key) {
      // Quit when ESC or 'q' is pressed
    case 27:
    case 'q':
    case 'Q':
      exit(0);
      break;
    // Add a point when '+' is pressed
    case '+':
        n += 1;
        break;

    // Remove a point when '+' is pressed
    case '-':
        n -= 1;
        break;

    // Run/Stop the Marching Cubes algorithm when 'm' is pressed
    case 'm':
    case 'M':
      setCUDA();
        break;

    // Change the surface to render when 'f' is pressed
    case 'f':
    case 'F':
      changeFunction();
        break;

    // Show/hide the grid when 'g' is pressed
    case 'g':
    case 'G':
      grid = 1-grid;
      break;

    // Show/hide the points when 'g' is pressed
    case 'p':
    case 'P':
      pts = 1-pts;
      break;

    // Show/hide the surface when 's' is pressed
    case 'x':
    case 'X':
      geom = 1-geom;
      break;
    case 'w':
      MainCamera.y += 0.1f;
      break;
    case 's':
      MainCamera.y -= 0.1f;
      break;
    case 'd':
      MainCamera.x += 0.1f;
      break;
    case 'a':
      MainCamera.x -= 0.1f;
      break;

	  default:
	      ;
  }
  deleteVBOs(vbo);
  createVBOs(vbo);
}

// GLUT calls this function when a mouse button is pressed
void mouse(int button, int state, int x, int y) {
  mouseX = x;
  mouseY = y;

  if (button == GLUT_LEFT_BUTTON) {
    if (state == GLUT_DOWN) {
      mouseLeftDown = 1;
    } else if (state == GLUT_UP)
      mouseLeftDown = 0;
  }

  else if (button == GLUT_RIGHT_BUTTON) {
    if (state == GLUT_DOWN) {
      mouseRightDown = 1;
    } else if (state == GLUT_UP)
      mouseRightDown = 0;
  }

  else if (button == GLUT_MIDDLE_BUTTON) {
    if (state == GLUT_DOWN) {
      mouseMiddleDown = 1;
    } else if (state == GLUT_UP)
      mouseMiddleDown = 0;
  }
}

// GLUT calls this function when mouse is moved while a button is held down
void motion(int x, int y) {
  if (mouseLeftDown) {
    MainCamera.pitch += (x - mouseX);
    MainCamera.roll += (y - mouseY);
    mouseX = x;
    mouseY = y;
  }
  if (mouseRightDown) {
    MainCamera.z -= (y - mouseY) * 0.2f;
    mouseY = y;
  }
}
//*****************************OpenGL initialization*************************
// Sets up projection and modelview matrices.
GLvoid initCamera() {
  // Set up the perspective matrix.
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // FOV, AspectRatio, NearClip, FarClip
  gluPerspective(58.0f, (float)(width) / height, 0.05f, 5.0f);

  // Set up the axis matrices.
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  // glRotatef(10,1,0,0);
  // glRotatef(20,0,0,1);
  // glTranslatef(-0,-0,-2);
}

// Sets up OpenGL lights.
GLvoid initLights() {
  // Define each color component.
  GLfloat ambient[] = {0.2f, 0.2f, 0.2f, 1.0f};
  GLfloat diffuse[] = {0.7f, 0.7f, 0.7f, 1.0f};
  GLfloat specular[] = {1.0f, 1.0f, 1.0f, 1.0f};

  // Set each color component.
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular);

  // Define and set position.
  float lightPos[4] = {0, 0, 20, 1};
  glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

  // Turn on lighting.
  glEnable(GL_LIGHT0);
  glEnable(GL_LIGHTING);
}

// Sets the OpenGL material state.
GLvoid initMaterial() {
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_COLOR_MATERIAL);
}

// Initialize OpenGL texture.
GLvoid initTexture() {
  glEnable(GL_TEXTURE_2D);
}

// Sets up OpenGL colors
GLvoid initColors() {
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glColor3f(0.0, 0.0, 0.0);
  glLineWidth(1.0);
  glPointSize(5.0);
}

// Sets up OpenGL state.
GLvoid initGL() {
  // Shading method: GL_SMOOTH or GL_FLAT
  glShadeModel(GL_SMOOTH);

  // Enable depth-buffer test.
  glEnable(GL_DEPTH_TEST);

  // Set the type of depth test.
  glDepthFunc(GL_LEQUAL);

  // 0 is near, 1 is far
  glClearDepth(1.0f);

  // Set camera settings.
  initCamera();

  // Set texture settings.
  initTexture();

  // Set lighting settings.
  initLights();

  // Set material settings.
  initMaterial();

  // Set color settings.
  initColors();
}

} // namespace DynaMap