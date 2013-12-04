#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>
#include <sstream>

#include "extra.h"
#include "camera.h"

///TODO: include more headers if necessary

#include "TimeStepper.h"
#include "simpleSystem.h"
#include "pendulumSystem.h"
#include "ClothSystem.h"

#define KB_UP 101
#define KB_DOWN 103
#define KB_LEFT 100
#define KB_RIGHT 102
#define KB_ESCAPE 27

using namespace std;

// Globals here.
namespace
{

    SimpleSystem *system;
    TimeStepper * timeStepper;
    // speed when pressing arrow keys
    const float SHIFT_SPEED = 1.0;

    float STEP_SIZE = 0.04f; //Works for RK4
    // float STEP_SIZE = 0.008f; //Works for Trapzoidal
    bool FOLLOW_MODE = false;
    bool FIRST_PERSON = false;
    int BIRD_POSITION_INDEX = 0;
    int numInitialBirds = 15;
    int numInitialPredators = 4;

  // initialize your particle systems
  ///TODO: read argv here. set timestepper , step size etc
  void initSystem(int argc, char * argv[])
  {
    // seed the random number generator with the current time
    srand( time( NULL ) );
    system = new SimpleSystem(numInitialBirds, numInitialPredators);
    //system = new PendulumSystem(3);
    // system = new ClothSystem();
    
    cout << "Running with Integrator: RK4" << endl;
    timeStepper = new RK4();

    cout << "Step size is: " << STEP_SIZE << "\n" << endl;
    cout << "*****************************************************************************************************" << endl;
    cout << "* To move the cloth and pendulum systems, press and hold the arrow keys                             *" << endl;
    cout << "* To toggle particles/birds, press p                                                                *" << endl;
    cout << "* To set a circular goal, press s                                                                   *" << endl;
    cout << "* To return the goal to default (user-controlled), press d                                          *" << endl;
    cout << "* To create or restart the FlockSystem, press 1                                                     *" << endl;
    cout << "* To toggle wind, press w                                                                           *" << endl;
    cout << "* To exit press Escape                                                                              *" << endl;
    cout << "*****************************************************************************************************" << endl;
  }


  // Take a step forward for the particle shower
  ///TODO: Optional. modify this function to display various particle systems
  ///and switch between different timeSteppers
  void stepSystem()
  {
    if(timeStepper!=0){
      timeStepper->takeStep(system,STEP_SIZE);
      system->step();
    }
  }

  // Draw the current particle positions
  void drawSystem()
  {
    // Base material colors (they don't change)
    GLfloat particleColor[] = {0.4f, 0.7f, 1.0f, 1.0f};
    GLfloat floorColor[] = {0.0f, 1.0f, 0.0f, 1.0f};
    GLfloat skyColor[] = {0.3f, 0.3f, 1.0f, 1.0f};
    
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, particleColor);
    
    glutSolidSphere(0.1f,10.0f,10.0f);
    system->draw();

    //background
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, floorColor);
    glPushMatrix();
    glTranslatef(0.0f,-5.0f,0.0f);
    glScaled(50.0f,0.01f,50.0f);
    glutSolidCube(1);
    glPopMatrix();

    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, skyColor);

    //top
    glPushMatrix();
    glTranslatef(0.0f,25.0f,0.0f);
    glRotatef(90,0.0f,1.0f,0.0f);
    glScaled(50.0f,0.01f,50.0f);
    glutSolidCube(1);
    glPopMatrix();


    glPushMatrix();
    glTranslatef(0.0f,0.0f,-25.0f);
    glRotatef(90,-1.0f,0.0f,0.0f);
    glScaled(50.0f,0.01f,50.0f);
    glutSolidCube(1);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0f,0.0f,25.0f);
    glRotatef(90,1.0f,0.0f,0.0f);
    glScaled(50.0f,0.01f,50.0f);
    glutSolidCube(1);
    glPopMatrix();

    

    glPushMatrix();
    glTranslatef(-25.0f,0.0f,0.0f);
    glRotatef(90,0.0f,0.0f,-1.0f);
    glScaled(50.0f,0.01f,50.0f);
    glutSolidCube(1);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(25.0f,0.0f,0.0f);
    glRotatef(90,0.0f,0.0f,1.0f);
    glScaled(50.0f,0.01f,50.0f);
    glutSolidCube(1);
    glPopMatrix();
  }
        

    //-------------------------------------------------------------------
    
        
    // This is the camera
    Camera camera;

    // These are state variables for the UI
    bool g_mousePressed = false;

    // Declarations of functions whose implementations occur later.
    void arcballRotation(int endX, int endY);
    void keyboardFunc( unsigned char key, int x, int y);
    void specialFunc( int key, int x, int y );
    void mouseFunc(int button, int state, int x, int y);
    void motionFunc(int x, int y);
    void reshapeFunc(int w, int h);
    void drawScene(void);
    void initRendering();

    // This function is called whenever a "Normal" key press is
    // received.
    void keyboardFunc( unsigned char key, int x, int y )
    {
        switch ( key )
        {
        case KB_ESCAPE: // Escape key
            exit(0);
            break;
        case ' ':
        {
            Matrix4f eye = Matrix4f::identity();
            camera.SetRotation( eye );
            camera.SetCenter( Vector3f::ZERO );
            break;
        }
        case '1':
            system = new SimpleSystem(numInitialBirds, numInitialPredators);
            break;
        case '2':
            // system = new PendulumSystem(4);
            break;
        case '3':
            // system = new ClothSystem();
            break;
        case 'd':
            cout << "Setting default goal" << endl;
            system->setGoalPattern(system->GOAL_DEFAULT);
            break;
        case 's':
            cout << "Setting circular goal" << endl;
            system->setGoalPattern(system->GOAL_CIRCULAR);
            break;
        case 'e':
           break;
        case 'r':
            break;
        case 't':
            break;
        case 'i':
            break;
        case 'p':
            cout << "Toggling particles rendering" << endl;
            system->toggleParticles();
            break;
        case 'w':
            cout << "Toggling wind" << endl;
            system->toggleWind();
            break;
        case 'b':
            cout << "Toggling follow mode"<< endl;
            FOLLOW_MODE = !FOLLOW_MODE;
            if (FOLLOW_MODE)
            {
                BIRD_POSITION_INDEX = system->getRandomBirdPositionIndex();
            }
            else
            {
                BIRD_POSITION_INDEX = 0;
            }
            break;
        case 'v':
            cout << "Following random bird" << endl;
            if (FOLLOW_MODE)
            {
                BIRD_POSITION_INDEX = system->getRandomBirdPositionIndex();
            }
            break;
        case 'c':
            cout << "Toggling first person mode" << endl;
            FIRST_PERSON = !FIRST_PERSON;
            if (FIRST_PERSON)
            {
                camera.SetDistance(0.05);
            }
            else
            {
                camera.SetDistance(10);
            }
            break;
        default:
            cout << "Unhandled key press " << key << "." << endl;        
        }

        glutPostRedisplay();
    }

    // This function is called whenever a "Special" key press is
    // received.  Right now, it's handling the arrow keys.
    void specialFunc( int key, int x, int y )
    {

        switch ( key )
        {
            case KB_UP:
                // cout << "shifting up" << endl;
                system->shiftRoot(Vector3f(0,SHIFT_SPEED,0));
                break;
            case KB_DOWN:
                // cout << "shifting down" << endl;
                system->shiftRoot(Vector3f(0, -1.0 * SHIFT_SPEED,0));
                break;
            case KB_LEFT:
                // cout << "shifting left" << endl;
                system->shiftRoot(Vector3f(-1.0 * SHIFT_SPEED,0,0));
                break;
            case KB_RIGHT:
                // cout << "shifting right" << endl;
                system->shiftRoot(Vector3f(SHIFT_SPEED,0,0));
                break;
            default:
                break;
        }
        //glutPostRedisplay();
    }

    //  Called when mouse button is pressed.
    void mouseFunc(int button, int state, int x, int y)
    {
        if (FIRST_PERSON)
        {
            return;
        }
        if (state == GLUT_DOWN)
        {
            g_mousePressed = true;
            
            switch (button)
            {
            case GLUT_LEFT_BUTTON:
                camera.MouseClick(Camera::LEFT, x, y);
                break;
            case GLUT_MIDDLE_BUTTON:
                camera.MouseClick(Camera::MIDDLE, x, y);
                break;
            case GLUT_RIGHT_BUTTON:
                camera.MouseClick(Camera::RIGHT, x,y);
            default:
                break;
            }                       
        }
        else
        {
            camera.MouseRelease(x,y);
            g_mousePressed = false;
        }
        glutPostRedisplay();
    }

    // Called when mouse is moved while button pressed.
    void motionFunc(int x, int y)
    {
        camera.MouseDrag(x,y);        
    
        glutPostRedisplay();
    }

    // Called when the window is resized
    // w, h - width and height of the window in pixels.
    void reshapeFunc(int w, int h)
    {
        camera.SetDimensions(w,h);

        camera.SetViewport(0,0,w,h);
        camera.ApplyViewport();

        // Set up a perspective view, with square aspect ratio
        glMatrixMode(GL_PROJECTION);

        camera.SetPerspective(50);
        glLoadMatrixf( camera.projectionMatrix() );
    }

    // Initialize OpenGL's rendering modes
    void initRendering()
    {
        glEnable(GL_DEPTH_TEST);   // Depth testing must be turned on
        glEnable(GL_LIGHTING);     // Enable lighting calculations
        glEnable(GL_LIGHT0);       // Turn on light #0.

        glEnable(GL_NORMALIZE);

        // Setup polygon drawing
        glShadeModel(GL_SMOOTH);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

        // Clear to black
        glClearColor(0,0,0,1);
    }

    // This function is responsible for displaying the object.
    void drawScene(void)
    {
        //Code to follow a bird
        if (FOLLOW_MODE)
        {
            camera.SetCenter(system->getPositionAtIndex(BIRD_POSITION_INDEX));
        }

        // Clear the rendering window
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glMatrixMode( GL_MODELVIEW );  
        glLoadIdentity();              

        // Light color (RGBA)
        GLfloat Lt0diff[] = {1.0,1.0,1.0,1.0};
        GLfloat Lt0pos[] = {3.0,3.0,5.0,1.0};
        glLightfv(GL_LIGHT0, GL_DIFFUSE, Lt0diff);
        glLightfv(GL_LIGHT0, GL_POSITION, Lt0pos);

        glLoadMatrixf( camera.viewMatrix() );

        // THIS IS WHERE THE DRAW CODE GOES.

        drawSystem();

        // This draws the coordinate axes when you're rotating, to
        // keep yourself oriented.
        if( g_mousePressed )
        {
            glPushMatrix();
            Vector3f eye = camera.GetCenter();
            glTranslatef( eye[0], eye[1], eye[2] );

            // Save current state of OpenGL
            glPushAttrib(GL_ALL_ATTRIB_BITS);

            // This is to draw the axes when the mouse button is down
            glDisable(GL_LIGHTING);
            glLineWidth(3);
            glPushMatrix();
            glScaled(5.0,5.0,5.0);
            glBegin(GL_LINES);
            glColor4f(1,0.5,0.5,1); glVertex3f(0,0,0); glVertex3f(1,0,0);
            glColor4f(0.5,1,0.5,1); glVertex3f(0,0,0); glVertex3f(0,1,0);
            glColor4f(0.5,0.5,1,1); glVertex3f(0,0,0); glVertex3f(0,0,1);

            glColor4f(0.5,0.5,0.5,1);
            glVertex3f(0,0,0); glVertex3f(-1,0,0);
            glVertex3f(0,0,0); glVertex3f(0,-1,0);
            glVertex3f(0,0,0); glVertex3f(0,0,-1);

            glEnd();
            glPopMatrix();

            glPopAttrib();
            glPopMatrix();
        }
                 
        // Dump the image to the screen.
        glutSwapBuffers();
    }

    void timerFunc(int t)
    {
        stepSystem();

        glutPostRedisplay();

        glutTimerFunc(t, &timerFunc, t);
    }

    

    
    
}

// Main routine.
// Set up OpenGL, define the callbacks and start the main loop
int main( int argc, char* argv[] )
{
    glutInit( &argc, argv );

    // We're going to animate it, so double buffer 
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );

    // Initial parameters for window position and size
    glutInitWindowPosition( 60, 60 );
    glutInitWindowSize( 600, 600 );

    camera.SetDimensions( 600, 600 );

    camera.SetDistance( 10 );
    camera.SetCenter( Vector3f::ZERO );

    glutCreateWindow("BodaciousBoids");

    // Initialize OpenGL parameters.
    initRendering();

    // Setup particle system
    initSystem(argc,argv);

    // Set up callback functions for key presses
    glutKeyboardFunc(keyboardFunc); // Handles "normal" ascii symbols
    glutSpecialFunc(specialFunc);   // Handles "special" keyboard keys

    // Set up callback functions for mouse
    glutMouseFunc(mouseFunc);
    glutMotionFunc(motionFunc);

    // Set up the callback function for resizing windows
    glutReshapeFunc( reshapeFunc );

    // Call this whenever window needs redrawing
    glutDisplayFunc( drawScene );

    // Trigger timerFunc every 20 msec
    glutTimerFunc(20, timerFunc, 20);
    
    cout << " Starting main loop " << endl;
    // Start the main loop.  glutMainLoop never returns.
    glutMainLoop();

    return 0;	// This line is never reached.
}
