// CS184 Simple OpenGL Example
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include "Eigen/Dense"
#include "arm.h"
#include "arm.cpp"
#include "jacobian.h"
#include "jacobian.cpp"
#include <time.h>
#include <math.h>

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265

using namespace std;
using namespace Eigen;

//****************************************************
// Some Classes
//****************************************************
class Viewport {
  public:
    int w, h; // width and height
};


//****************************************************
// Global Variables
//****************************************************
Viewport viewport;
Arm arm = Arm::Arm();
float lambda = 10;
Vector3f goal(.25, .25, .7);


//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
  viewport.w = w;
  viewport.h = h;

  glViewport(0,0,viewport.w,viewport.h);// sets the rectangle that will be the window
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();                // loading the identity matrix for the screen

  //----------- setting the projection -------------------------
  // glOrtho sets left, right, bottom, top, zNear, zFar of the chord system


  // glOrtho(-1, 1 + (w-400)/200.0 , -1 -(h-400)/200.0, 1, 1, -1); // resize type = add
  // glOrtho(-w/400.0, w/400.0, -h/400.0, h/400.0, 1, -1); // resize type = center

  // gluLookAt(0, .5, -1, 0, 0, 0, 0, 1, 0);
  glOrtho(-1, 1, -1, 1, 1, -1);    // resize type = stretch

  //------------------------------------------------------------
}


//****************************************************
// sets the window up
//****************************************************
void initScene() {
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Clear to black, fully transparent

  myReshape(viewport.w,viewport.h);
}

//***************************************************
// function that does the actual drawing
//***************************************************
void myDisplay() {


  //----------------------- ----------------------- -----------------------
  // This is a quick hack to add a little bit of animation.
  static float tip = 0.5f;
  const  float stp = 0.0001f;
  const  float beg = 0.1f;
  const  float end = 0.9f;

  tip += stp;
  if (tip>end) tip = beg;
  //----------------------- ----------------------- -----------------------


  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                // clear the color buffer (sets everything to black)

  glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
  glLoadIdentity();                           // make sure transformation is "zero'd"


  //----------------------- code to draw objects --------------------------

  GLUquadric *quad = gluNewQuadric();

  glEnable ( GL_COLOR_MATERIAL ) ;

  glPushMatrix();
  glColorMaterial ( GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE ) ;
  glColor3f(1, 0, 0);
  glTranslatef(goal(0), goal(1), goal(2));
  gluSphere(quad, .02, 32, 20);
  glPopMatrix();

  gluDeleteQuadric(quad);

  float tolerance = 0.01;
  // float lambda = 10; // moved to global variable

  vector<float> lengths = arm.getLengthVector();
  // vector<Vector3f> rotationsV = arm.getRotationVector();
  vector<MatrixXf> rotationsM = arm.getRotationMatrices();
  // Vector3f endEffector = Jacobian::endEffector(lengths, rotationsV);
  Vector3f endEffector = arm.getEndEffector();


  arm.drawArm();

  if ((endEffector - goal).norm() >= tolerance) {
    MatrixXf jac = Jacobian::jacobianMatrices(lengths, rotationsM);
    MatrixXf pseudoInv = Jacobian::pseudoInverse(jac);
    VectorXf dr = pseudoInv * (lambda * (goal - endEffector));

    arm.updateRotations(dr);
    if ((endEffector - goal).norm() >= arm.error) {
      lambda *= .25;
    }
    arm.error = (endEffector - goal).norm();
  }

  glLoadIdentity();



  glEnable(GL_DEPTH_TEST);

  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);

  GLfloat diffuse0[]={1.0, 1.0, 1.0, 0.0};
  GLfloat ambient0[]={0.0, 0.0, 0.0, 0.0};
  GLfloat specular0[]={1.0, 1.0, 1.0, 0.0};
  GLfloat light0_pos[]={-5.0, 0.0, -3.0, 0.0};

  glEnable(GL_LIGHT0);
  glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);
  glEnd();
  
  //-----------------------------------------------------------------------

  glFlush();
  glutSwapBuffers();                           // swap buffers (we earlier set double buffer)
}


//****************************************************
// called by glut when there are no messages to handle
//****************************************************
void myFrameMove() {
  //nothing here for now
#ifdef _WIN32
  Sleep(10);                                   //give ~10ms back to OS (so as not to waste the CPU)
#endif
  glutPostRedisplay(); // forces glut to call the display function (myDisplay())
}


//****************************************************
// MOUSE CLICKING FUNCTIONALITY THING
//****************************************************
void mouseClick(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON and state == GLUT_DOWN) {
    float xpos = (x - viewport.w / 2.0) / (viewport.w / 2);
    float ypos =  -(y - viewport.h / 2.0) / (viewport.h / 2);
    goal << xpos, ypos, 0;
    lambda = 10;
  }
}

void SpecialInput(int key, int x, int y) {
  switch(key)
  {
  case GLUT_KEY_UP:
    goal(2) += .5;
    break;
  case GLUT_KEY_DOWN:
    goal(2) -= .5;
    break;
  }
// glutPostRedisplay();
}


//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
  //This initializes glut
  glutInit(&argc, argv);

  //This tells glut to use a double-buffered window with red, green, and blue channels 
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);

  // Initalize theviewport size
  viewport.w = 400;
  viewport.h = 400;

  //The size and position of the window
  glutInitWindowSize(viewport.w, viewport.h);

  // glutInitWindowPosition(0, 0);
  glutCreateWindow("CS184!");

  initScene();                                 // quick function to set up scene

  // glMatrixMode(GL_PROJECTION);
  // glLoadIdentity();
  // glOrtho(-3.5, 3.5, -3.5, 3.5, 5, -5);
  // glMatrixMode(GL_MODELVIEW);
  // glLoadIdentity();

  // glEnable(GL_DEPTH_TEST);  // enable z-buffering
  // glDepthFunc(GL_LESS);

  glutSpecialFunc(SpecialInput);
  glutMouseFunc(mouseClick);
  glutDisplayFunc(myDisplay);                  // function to run when its time to draw something
  glutReshapeFunc(myReshape);                  // function to run when the window gets resized

  // glEnable(GL_DEPTH_TEST);

  glutIdleFunc(myDisplay);                   // function to run when not handling any other task

  glutMainLoop();                              // infinite loop that will keep drawing and resizing and whatever else


  return 0;
}








