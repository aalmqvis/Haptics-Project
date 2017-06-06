//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2014, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.0.0 $Rev: 1292 $
*/
//==============================================================================
//#define _GLIBCXX_USE_CXX11_ABI 0

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a virtual object
//cMultiMesh* object;
//cMultiMesh* object_2;
//cMultiMesh* object_3;

cShapeSphere* object;
cShapeSphere* object_2;
cShapeSphere* object_3;

// rendering option
bool showTexture = true;
bool showNormals = false;
bool showWireMode = false;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
// 0 added
cGenericHapticDevicePtr hapticDevice = 0;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = true;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// root resource path
string resourceRoot;

// added
cVector3d newPosition;
cVector3d hapticDevicePosition;

// A 3D cursor for the haptic device
cShapeSphere* m_cursor;

// Material properties used to render the color of the cursors
cMaterialPtr m_matCursorColor;

float G;
float massM_cursor;
float massEarth;
float massPlanet2;
float massPlanet3;

float radiusM_cursorEarth;
float radiusM_cursorPlanet2;
float radiusM_cursorPlanet3;

float newtonsGravitationalForceMagnitude;
float newtonsGravitationalForceMagnitudePlanet2;
float newtonsGravitationalForceMagnitudePlanet3;

cVector3d newtonsGravitationalForceDirection;
cVector3d newtonsGravitationalForceDirectionPlanet2;
cVector3d newtonsGravitationalForceDirectionPlanet3;

cVector3d gravityForceM_cursorEarth;
cVector3d gravityForceM_cursorPlanet2;
cVector3d gravityForceM_cursorPlanet3;

float m_cursorRadius;

// A label used to demonstrate debug output
cLabel* m_debugLabel1;
cLabel* m_debugLabel2;
cLabel* m_debugLabel3;
cLabel* m_debugLabel4;
cLabel* m_debugLabel5;
cLabel* m_debugLabel6;
cLabel* m_debugLabel7;
cLabel* m_debugLabel8;
cLabel* m_titleLabel;

cVector3d lastForceBeforeCollsion;
float currentForce;
float currentForce_2;
float currentForce_3;


cVector3d angVel;
cVector3d angVelPlanet2;
cVector3d angVelPlanet3;

cVector3d linVel;
cVector3d linVelPlanet2;
cVector3d linVelPlanet3;

bool useHaptics_object;
bool useHaptics_object_2;
bool useHaptics_object_3;
bool useHaptics_allObjects;

cVector3d noForce;

cVector3d objectStartPos;
cVector3d object_2StartPos;
cVector3d object_3StartPos;

std::stringstream titleLabel;

bool task1;
bool startScreen;
cVector3d collisionForce;
float constantK;

cImage* formula;
//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
// commented out
//#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);


//==============================================================================
/*
    DEMO:    space.cpp

    This demonstration illustrate an object floating in free space.The user
    can interact wth the model with the haptic device.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 19-space" << endl;
    cout << "Copyright 2003-2014" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[x] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = 0.8 * screenH;
    windowH = 0.5 * screenH;
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY;

    // initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    }
    else
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    }

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);
    glewInit();

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (5.0, 0.0, 0.6),    // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),    // lookat position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front/behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cDirectionalLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

    // set lighting conditions
    light->m_ambient.set(0.4, 0.4, 0.4);
    light->m_diffuse.set(0.8, 0.8, 0.8);
    light->m_specular.set(1.0, 1.0, 1.0);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // added
    // Read the number of haptic devices currently connected to the computer
    int numHapticDevices = handler->getNumDevices();
    cout << numHapticDevices << endl;
    // If there is at least one haptic device detected...
    if (numHapticDevices)
    {
        // Get a handle to the first haptic device
        handler->getDevice(hapticDevice);

        // Open connection to haptic device
        hapticDevice->open();

        // Calbrate if needed
        hapticDevice->calibrate();

        // Retrieve information about the current haptic device
        //cHapticDeviceInfo info = hapticDevice->getSpecifications();
    }
    // end of added

    // get access to the first available haptic device
    //handler->getDevice(hapticDevice, 0);
    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // if the haptic devices carries a gripper, enable it to behave like a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    // create a 3D tool and add it to the world
    tool = new cToolCursor(world);
    //world->addChild(tool);

    // Create a cursor with its radius set
    m_cursorRadius = 0.08;
    m_cursor = new cShapeSphere(m_cursorRadius);
    // Add cursor to the world
    world->addChild(m_cursor);

    m_matCursorColor = cMaterialPtr(new cMaterial());
    m_matCursorColor->m_ambient.set(0.1, 0.1, 0.4);
    m_matCursorColor->m_diffuse.set(0.3, 0.4, 0.2);
    m_matCursorColor->m_specular.set(0.3, 0.3, 0.3);
    m_cursor->m_material = m_matCursorColor;


    // connect the haptic device to the tool
    tool->setHapticDevice(hapticDevice);

    // define the radius of the tool (sphere)
    double toolRadius = 0.1;

    // define a radius for the tool
    tool->setRadius(toolRadius);

    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);

    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();

    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(true);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);


    // initialize tool by connecting to haptic device
    tool->start();

    useHaptics_object = true;

    //--------------------------------------------------------------------------
    // CREATE OBJECT
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create a virtual mesh
    //object = new cMultiMesh();
    //object_2 = new cMultiMesh();
    object = new cShapeSphere(0.08);
    object_2 = new cShapeSphere(0.08);
    //object_3 = new cMultiMesh();
    object_3 = new cShapeSphere(0.08);


    formula = new cImage();
    formula->loadFromFile("formulaImage.png");
    //world->addChild(formula);
    //camera->m_frontLayer->addChild(formula)
    //formula->setLocalPos(windowW*0.4, windowH*0.8);

    m_matCursorColor = cMaterialPtr(new cMaterial());
    m_matCursorColor->m_ambient.set(0.2, 0.1, 0.1);
    m_matCursorColor->m_diffuse.set(1, 0.1, 0.1);
    m_matCursorColor->m_specular.set(1, 0.1, 0.1);
    object->m_material = m_matCursorColor;

    m_matCursorColor = cMaterialPtr(new cMaterial());
    m_matCursorColor->m_ambient.set(0.2, 0.1, 0.1);
    m_matCursorColor->m_diffuse.set(1, 0.1, 0.1);
    m_matCursorColor->m_specular.set(1, 0.1, 0.1);
    object_2->m_material = m_matCursorColor;

    m_matCursorColor = cMaterialPtr(new cMaterial());
    m_matCursorColor->m_ambient.set(0.2, 0.1, 0.1);
    m_matCursorColor->m_diffuse.set(0.1, 1, 0.1);
    m_matCursorColor->m_specular.set(0.1, 1, 0.1);
    object_3->m_material = m_matCursorColor;

    // add object to world
    world->addChild(object);
    world->addChild(object_2);
    world->addChild(object_3);


    // set the position of the object at the center of the world
    objectStartPos = cVector3d(-0.8,-1,0.1);
    object->setLocalPos(objectStartPos);

    object_2StartPos = cVector3d(-0.4,0.8,-0.7);
    object_2->setLocalPos(object_2StartPos);

    object_3StartPos = cVector3d(-0.7,-0.8,0.29);
    object_3->setLocalPos(object_3StartPos);


    // rotate the object 90 degrees
    object->rotateAboutGlobalAxisDeg(cVector3d(0,0,1), 90);
    object_2->rotateAboutGlobalAxisDeg(cVector3d(0,0,1), 90);
    object_3->rotateAboutGlobalAxisDeg(cVector3d(0,0,1), 90);

    // load an object file
    //bool fileload;
    //fileload = object->loadFromFile("LowPolyEarth.3ds");
    //fileload = object_2->loadFromFile("LowPolyEarth_red.3ds");
    //fileload = object_3->loadFromFile("LowPolyEarth.3ds");

    //if (!fileload)
    //{
//        cout << "Error - 3D Model failed!!! to load correctly." << endl;
//        close();
//        return (-1);
//    }

    // compute a boundary box
    //object->computeBoundaryBox(true);
    //object_2->computeBoundaryBox(true);
    //object_3->computeBoundaryBox(true);

    // get dimensions of object
    //double size = cSub(object->getBoundaryMax(), object->getBoundaryCenter()).length();
    //double size_2 = cSub(object_2->getBoundaryMax(), object_2->getBoundaryCenter()).length();
    //double size_3 = cSub(object_3->getBoundaryMax(), object_3->getBoundaryCenter()).length();


    //cout << "size" << endl;
    //cout << size << endl;
    // resize object to screen
   // if (size > 0)
    //{
        //object->scale(0.15 * tool->getWorkspaceRadius() / size);
        //object_2->scale(0.15 * tool->getWorkspaceRadius() / size_2);
        //object_3->scale(0.15 * tool->getWorkspaceRadius() / size_3);
    //}

    // compute collision detection algorithm
    //object->createAABBCollisionDetector(toolRadius);
    //object_2->createAABBCollisionDetector(toolRadius);
    //object_3->createAABBCollisionDetector(toolRadius);

    cMaterial mat;
    mat.setHapticTriangleSides(true, true);
    object->setMaterial(mat);
    object_2->setMaterial(mat);
    object_3->setMaterial(mat);

    // Maybe texture is unecessary?

    // define some environmental texture mapping
    cTexture2dPtr texture = cTexture2d::create();

    // load texture file
    //fileload = texture->loadFromFile("chrome.jpg");


    // enable spherical mapping
    //texture->setSphericalMappingEnabled(true);

    // assign texture to object
    //object->setTexture(texture, true);
    //object_2->setTexture(texture, true);
    //object_3->setTexture(texture, true);

    // enable texture mapping
    //object->setUseTexture(true, true);
    //object_2->setUseTexture(true, true);
    //object_3->setUseTexture(true, true);

    // disable culling
    //object->setUseCulling(false, true);
    //object_2->setUseCulling(false, true);
    //object_3->setUseCulling(false, true);

    // enable display list for faster graphic rendering
    //object->setUseDisplayList(true);
    //object_2->setUseDisplayList(true);
    //object_3->setUseDisplayList(true);

    // define a default stiffness for the object
    //object->setStiffness(maxStiffness, true);
    //object_2->setStiffness(maxStiffness, true);
    //object_3->setStiffness(maxStiffness, true);

    // define some haptic friction properties
    //object->setFriction(0.1, 0.2, true);
    //object_2->setFriction(0.1, 0.2, true);
    //object_3->setFriction(0.1, 0.2, true);



    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // Create a label used to show how debug output can be handled
    m_debugLabel1 = new cLabel(font);
    m_debugLabel2 = new cLabel(font);
    m_debugLabel3 = new cLabel(font);
    m_debugLabel4 = new cLabel(font);
    m_debugLabel5 = new cLabel(font);
    m_debugLabel6 = new cLabel(font);
    m_debugLabel7 = new cLabel(font);
    m_debugLabel8 = new cLabel(font);
    m_titleLabel = new cLabel(font);

    // Labels need to be added to the camera instead of the world
    camera->m_frontLayer->addChild(m_titleLabel);
    camera->m_frontLayer->addChild(m_debugLabel1);
    camera->m_frontLayer->addChild(m_debugLabel2);
    camera->m_frontLayer->addChild(m_debugLabel3);
    camera->m_frontLayer->addChild(m_debugLabel4);
    camera->m_frontLayer->addChild(m_debugLabel5);
    camera->m_frontLayer->addChild(m_debugLabel6);
    camera->m_frontLayer->addChild(m_debugLabel7);
    camera->m_frontLayer->addChild(m_debugLabel8);



    //camera->m_frontLayer->addChild(m_titleLabel);
    world->addChild(m_titleLabel);
    m_titleLabel->m_fontColor.setWhite();
    /*
    m_debugLabel2->m_fontColor.setWhite();
    //m_debugLabel3->m_fontColor.set(0.8,0.3,0.2);
    m_debugLabel3->m_fontColor.setWhite();

    m_debugLabel4->m_fontColor.setWhite();
    m_debugLabel5->m_fontColor.setWhite();
    */
    // create a label to display the haptic rate of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setWhite();
    //camera->m_frontLayer->addChild(labelHapticRate);

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.0, 1.0, 1.0),
                                     cColorf(1.0, 1.0, 1.0),
                                     cColorf(0.8, 0.8, 0.8),
                                     cColorf(0.8, 0.8, 0.8));

    // load a texture file
   // fileload = background->loadFromFile("earth.jpg");
    //background->loadFromFile("formulaImage.png");

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    object->setEnabled(false);
    object_2->setEnabled(false);
    object_3->setEnabled(false);

    useHaptics_object = false;
    useHaptics_object_2 = false;
    useHaptics_object_3 = false;

    startScreen = true;

    // added
    // Simulation in now running
    simulationRunning = true;

    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // start the main graphics rendering loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    // close everything
    close();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        // close everything
        close();

        // exit application
        exit(0);
    }

    if (key == '0'){

        object->setEnabled(false);
        object_2->setEnabled(false);
        object_3->setEnabled(false);

        useHaptics_object = false;
        useHaptics_object_2 = false;
        useHaptics_object_3 = false;

    }


    // option 1: show/hide texture
    /*if (key == 's')
    {
        //showTexture = !showTexture;
        //object->setUseTexture(showTexture);


        angVel.zero();
        angVelPlanet2.zero();
        angVelPlanet3.zero();

        linVel.zero();
        linVelPlanet2.zero();
        linVelPlanet3.zero();
    }
    */
    if(key == '1'){

        object->setEnabled(true);
        object_2->setEnabled(false);
        object_3->setEnabled(false);

        object->setLocalPos(objectStartPos);

        useHaptics_object = true;
        useHaptics_object_2 = false;
        useHaptics_object_3 = false;

        angVel.zero();
        angVelPlanet2.zero();
        angVelPlanet3.zero();

        linVel.zero();
        linVelPlanet2.zero();
        linVelPlanet3.zero();

        gravityForceM_cursorPlanet2 = cVector3d(0.0f,0.0f,0.0f);
        gravityForceM_cursorPlanet3 = cVector3d(0.0f,0.0f,0.0f);
        task1 = true;
        startScreen = false;
    }

    // option 2: wire/fill triangle mode
    if (key == '2'){

       // useHaptics_allObjects = false;
        object->setEnabled(false);
        object_2->setEnabled(true);
        object_3->setEnabled(true);

        object_2->setLocalPos(object_2StartPos);
        object_3->setLocalPos(object_3StartPos);

        useHaptics_object = false;
        useHaptics_object_2 = true;
        useHaptics_object_3 = true;

        angVel.zero();
        angVelPlanet2.zero();
        angVelPlanet3.zero();

        linVel.zero();
        linVelPlanet2.zero();
        linVelPlanet3.zero();

        task1 = false;
        startScreen = false;
    }

    if (key == 'q')
    {
        object->setEnabled(true);
        object_2->setEnabled(false);
        object_3->setEnabled(false);

        object->setLocalPos(objectStartPos);

        useHaptics_object = false;
        useHaptics_object_2 = false;
        useHaptics_object_3 = false;

        angVel.zero();
        angVelPlanet2.zero();
        angVelPlanet3.zero();

        linVel.zero();
        linVelPlanet2.zero();
        linVelPlanet3.zero();

        gravityForceM_cursorPlanet2 = cVector3d(0.0f,0.0f,0.0f);
        gravityForceM_cursorPlanet3 = cVector3d(0.0f,0.0f,0.0f);

        task1 = true;
        startScreen = false;
    }

    if (key == 'w')
    {
        object->setEnabled(false);
        object_2->setEnabled(true);
        object_3->setEnabled(true);

        object_2->setLocalPos(object_2StartPos);
        object_3->setLocalPos(object_3StartPos);

        useHaptics_object = false;
        useHaptics_object_2 = false;
        useHaptics_object_3 = false;

        angVel.zero();
        angVelPlanet2.zero();
        angVelPlanet3.zero();

        linVel.zero();
        linVelPlanet2.zero();
        linVelPlanet3.zero();

        task1 = false;
        startScreen = false;
    }

    if (key == '5')
    {

    }

    // reset scene
    if (key == 'r'){

        object->setLocalPos(objectStartPos);
        object_2->setLocalPos(object_2StartPos);
        object_3->setLocalPos(object_3StartPos);

        angVel.zero();
        angVelPlanet2.zero();
        angVelPlanet3.zero();

        linVel.zero();
        linVelPlanet2.zero();
        linVelPlanet3.zero();
    }

    if(key == 'h'){
        useHaptics_object = false;
        useHaptics_object_2 = false;
        useHaptics_object_3 = false;
    }

    // option f: toggle fullscreen
    if (key == 'f')
    {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }

    // option m: toggle vertical mirroring
    if (key == 'm')
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    // inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic rate label
    labelHapticRate->setText ("haptic rate: "+cStr(frequencyCounter.getFrequency(), 0) + " [Hz]");

    // update position of label
    //labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);





    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;

    // added
    // Inform the GLUT window to call updateGraphics again (next frame)
    if (simulationRunning)
    {
        glutPostRedisplay();
    }
}

//------------------------------------------------------------------------------

// added
enum cMode
{
    IDLE,
    SELECTION
};

void updateHaptics(void)
{   
    // angular velocity
    angVel = cVector3d(0.0f,0.0f,0.1f);
    angVelPlanet2 = cVector3d(0.0f,0.0f,0.1f);
    angVelPlanet3 = cVector3d(0.0f,0.0f,0.1f);

    linVel = cVector3d(0.0f,-0.1f,-0.0f);
    linVelPlanet2 = cVector3d(0.0f,-0.1f,-0.0f);
    linVelPlanet3 = cVector3d(0.0f,-0.1f,-0.0f);

    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // reset tool
    tool->initialize();
    /*
    std::stringstream ss1;
    std::stringstream ss2;
    std::stringstream ss3;
    std::stringstream ss4;
    std::stringstream ss5;

    ss1 << "" << endl;
    ss2 << "" << endl;
    ss3 << "" << endl;
    ss4 << "" << endl;
    ss5 << "" << endl;

    m_debugLabel1->setText(ss1.str());
    m_debugLabel2->setText(ss2.str());
    m_debugLabel3->setText(ss3.str());
    m_debugLabel4->setText(ss4.str());
    m_debugLabel5->setText(ss5.str());*/

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

        // restart the simulation clock
        clock.reset();
        clock.start();

        // update frequency counter
        frequencyCounter.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        cVector3d newPosition;
        hapticDevice->getPosition(newPosition);


        //newtonsGravitationForce
        //m_cursor->setLocalPos(newPosition);

        // update global variable for graphic display update
        //tool->();
        tool->updateToolImagePosition();

        // Update position and orientation of cursor
        //tool->setLocalPos(newPosition);
        //tool->setLocalPos();

        // compute interaction forces
        //tool->computeInteractionForces();
        //cout << newPosition << endl;

        // send forces to haptic device
        //tool->setForce();
        //tool->applyForces();
        //tool->applyToDevice();

        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////

        // get position of cursor in global coordinates
        cVector3d toolPos = tool->getDeviceGlobalPos();
        toolPos = toolPos * 3;
        tool->updateFromDevice();
        //m_cursor->setLocalPos(toolPos);

        // added
        G = 1.2;
        massM_cursor = 4;
        massEarth = 1;
        massPlanet2 = 0.5;
        massPlanet3 = 4;

        // get position of object in global coordinates
        cVector3d objectPos = object->getGlobalPos();        
        cVector3d object_2Pos = object_2->getGlobalPos();
        cVector3d object_3Pos = object_3->getGlobalPos();

        // compute a vector from the center of mass of the object (point of rotation) to the tool
        cVector3d v = cSub(toolPos, objectPos);
        cVector3d v_2 = cSub(toolPos, object_2Pos);
        cVector3d v_3 = cSub(toolPos, object_3Pos);

        radiusM_cursorEarth = sqrt( pow(v.x(),2) + pow(v.y(),2) + pow(v.z(),2) );
        radiusM_cursorPlanet2 = sqrt( pow(v_2.x(),2) + pow(v_2.y(),2) + pow(v_2.z(),2) );
        radiusM_cursorPlanet3 = sqrt( pow(v_3.x(),2) + pow(v_3.y(),2) + pow(v_3.z(),2) );

        newtonsGravitationalForceMagnitude = G*massM_cursor*massEarth/pow(radiusM_cursorEarth,2);
        newtonsGravitationalForceMagnitudePlanet2 = G*massM_cursor*massPlanet2/pow(radiusM_cursorPlanet2,2);
        newtonsGravitationalForceMagnitudePlanet3 = G*massM_cursor*massPlanet3/pow(radiusM_cursorPlanet3,2);

        newtonsGravitationalForceDirection = v/radiusM_cursorEarth;
        newtonsGravitationalForceDirectionPlanet2 = v_2/radiusM_cursorPlanet2;
        newtonsGravitationalForceDirectionPlanet3 = v_3/radiusM_cursorPlanet3;

        gravityForceM_cursorEarth = cNegate(newtonsGravitationalForceMagnitude*newtonsGravitationalForceDirection);
        gravityForceM_cursorPlanet2 = cNegate(newtonsGravitationalForceDirectionPlanet2)*newtonsGravitationalForceMagnitudePlanet2;
        gravityForceM_cursorPlanet3 = cNegate(newtonsGravitationalForceDirectionPlanet3)*newtonsGravitationalForceMagnitudePlanet3;

        float newtonsGravitationalForceMagnitudePlanet2UNTOUCHED = newtonsGravitationalForceMagnitudePlanet2;
        float newtonsGravitationalForceMagnitudePlanet3UNTOUCHED = newtonsGravitationalForceMagnitudePlanet3;

        // print newtons gravitational law force magnitude
        //cout << "newtonsGravitationalForceMagnitude " << endl;
        //cout << newtonsGravitationalForceMagnitude << endl;

        // print newtons gravitational law radius
        //cout << "radiusM_cursorEarth " << endl;
        //cout << radiusM_cursorEarth << endl;

        // compute angular acceleration based on the interaction forces
        // between the tool and the object
        cVector3d angAcc(0,0,0);
        cVector3d angAccPlanet2(0,0,0);
        cVector3d angAccPlanet3(0,0,0);

        cVector3d linAcc(0,0,0);
        cVector3d linAccPlanet2(0,0,0);
        cVector3d linAccPlanet3(0,0,0);

        cVector3d forceTool(0,0,0);
        cVector3d forceToolPlanet2(0,0,0);
        cVector3d forceToolPlanet3(0,0,0);


        // compute force that centers object at origin
        //cVector3d force = 2 * newtonsGravitationalForceDirection;
        // addedcVector3d gravityForceM_cursorEarth
       // cVector3d pulledToProxy = toolPos-objectPos;

        // Also defined further down
        float earthRadius = 0.10;
        float constraintDistance = earthRadius+m_cursorRadius;
        float y2 = 10*G*massM_cursor*massEarth/pow(constraintDistance,2);

        if (newtonsGravitationalForceMagnitude > 22){
            newtonsGravitationalForceMagnitude = 22;
        }
        if (newtonsGravitationalForceMagnitudePlanet2 > 14){
            newtonsGravitationalForceMagnitudePlanet2 = 14;
        }
        if (newtonsGravitationalForceMagnitudePlanet3 > 22){
            newtonsGravitationalForceMagnitudePlanet3 = 22;
        }

        cVector3d force = newtonsGravitationalForceDirection * newtonsGravitationalForceMagnitude;
        cVector3d forcePlanet2 = newtonsGravitationalForceDirectionPlanet2 * newtonsGravitationalForceMagnitudePlanet2;
        cVector3d forcePlanet3 = newtonsGravitationalForceDirectionPlanet3 * newtonsGravitationalForceMagnitudePlanet3;

        //cVector3d force = 10 * pulledToProxy;



        if (v.length() > 0.0 && v.length() < 10)
        {
            // get the last force applied to the cursor in global coordinates
            // we negate the result to obtain the opposite force that is applied on the
            // object
            //tool->getDeviceGlobalForce() = tool->getDeviceGlobalForce() + gravityForceM_cursorEarth;
            forceTool = cNegate(tool->getDeviceGlobalForce());

            //newtonsGravitationalForce = G*massProxy*massEarth/pow(radiusm_cursorEarth,2);
            //forceTool = cNegate(tool->m_hapticPoint->getLastComputedForce());
            //forceTool = 10*forceTool;
            //forceTool = cNegate(tool->m_lastComputedGlobalForce);

            // compute the effective force that contributes to rotating the object.
            cVector3d f = forceTool - cProject(forceTool, v);

            // compute the resulting torque
            cVector3d torque = cMul(v.length(), cCross( cNormalize(v), f));


            // update angular acceleration
            const double INERTIA = 0.4;
            angAcc = (1.0 / INERTIA) * torque;

        }

        if (v_2.length() > 0.0)
        {
            forceToolPlanet2 = cNegate(tool->getDeviceGlobalForce());

            cVector3d fPlanet2 = forceToolPlanet2 - cProject(forceToolPlanet2, v_2);

            // compute the resulting torque
            cVector3d torquePlanet2 = cMul(v_2.length(), cCross( cNormalize(v_2), fPlanet2));

            // update angular acceleration
            const double INERTIA = 0.4;
            angAccPlanet2 = (1.0 / INERTIA) * torquePlanet2;
        }

        if (v_3.length() > 0.0)
        {
            forceToolPlanet3 = cNegate(tool->getDeviceGlobalForce());

            cVector3d fPlanet3 = forceToolPlanet3 - cProject(forceToolPlanet3, v_3);

            // compute the resulting torque
            cVector3d torquePlanet3 = cMul(v_3.length(), cCross( cNormalize(v_3), fPlanet3));

            // update angular acceleration
            const double INERTIA = 0.4;
            angAccPlanet3 = (1.0 / INERTIA) * torquePlanet3;
        }


        /*
        if (newtonsGravitationalForceMagnitudePlanet2 > 10){
            forcePlanet2 = newtonsGravitationalForceDirectionPlanet2*10;
        }

        if (newtonsGravitationalForceMagnitudePlanet3 > 10){
            forcePlanet3 = newtonsGravitationalForceDirectionPlanet3*10;
        }
        */

        // ############### y2 nad constraintDistance DEFINED FURTHER UP
        //float constraintDistance = earthRadius+m_cursorRadius;

        //float y2 = G*massM_cursor*massEarth/pow(constraintDistance,2);
        //float y2 = 10;
        float y1 = -50; // arbitrary force deciding how much you can push into the planet
        float x2 = constraintDistance;
        float x1 = 0.05;
        float kconstant = (y2-y1)/(x2-x1);
        float mconstant = y1-kconstant*x1;
        cout << kconstant << endl;

        /*
        if (v.length() <= constraintDistance){
           newtonsGravitationalForceMagnitude = kconstant*v.length()+mconstant;
           std::stringstream ss5;
           ss5 << "newtonsGravitationalForceMagnitude" << newtonsGravitationalForceMagnitude << endl;
           gravityForceM_cursorEarth = cNegate(newtonsGravitationalForceMagnitude*newtonsGravitationalForceDirection);
        }
        */

        // working quite well, an invisible sphere around the earth
        /*if (v.length() < 1 ){
            newtonsGravitationalForceMagnitude = 2*v.length()-1;
            //force = newtonsGravitationalForceDirection * newtonsGravitationalForceMagnitude;
            gravityForceM_cursorEarth = cNegate(newtonsGravitationalForceMagnitude*newtonsGravitationalForceDirection);
        }*/
        // version 2 (invis sphere)
        // Earth
        if (v.length() < 0.2 ){
            newtonsGravitationalForceMagnitude = 120*v.length()-2;
            gravityForceM_cursorEarth = cNegate(newtonsGravitationalForceMagnitude*newtonsGravitationalForceDirection);
        }
        // Planet 2
        if (v_2.length() < 0.2 ){
            newtonsGravitationalForceMagnitudePlanet2 = 80*v_2.length()-2;
            gravityForceM_cursorPlanet2= cNegate(newtonsGravitationalForceMagnitudePlanet2*newtonsGravitationalForceDirectionPlanet2);
        }
        // Planet 3
        if (v_3.length() < 0.2 ){
            newtonsGravitationalForceMagnitudePlanet3 = 120*v_3.length()-2;
            gravityForceM_cursorPlanet3 = cNegate(newtonsGravitationalForceMagnitudePlanet3*newtonsGravitationalForceDirectionPlanet3);
        }

        // update linear acceleration
        const double MASS1 = 1*(4);
        const double MASS2 = 0.5*(4);
        const double MASS3 = 4*(4);

        linAcc = (1.0 / MASS1) * (force + forceTool);
        linAccPlanet2 = (1.0 / MASS2) * (forcePlanet2 + forceToolPlanet2);
        linAccPlanet3 = (1.0 / MASS3) * (forcePlanet3 + forceToolPlanet3);

        // update angular velocity
        angVel.add(timeInterval * angAcc);
        angVelPlanet2.add(timeInterval * angAccPlanet2);
        angVelPlanet3.add(timeInterval * angAccPlanet3);

        // update linear velocity
        linVel.add(timeInterval * linAcc);
        linVelPlanet2.add(timeInterval * linAccPlanet2);
        linVelPlanet3.add(timeInterval * linAccPlanet3);

        // set a threshold on the rotational velocity term
        const double MAX_ANG_VEL = 10.0;
        double vel = angVel.length();
        double velPlanet2 = angVelPlanet2.length();
        double velPlanet3 = angVelPlanet3.length();

        if (vel > MAX_ANG_VEL)
        {
            angVel.mul(MAX_ANG_VEL / vel);
        }
        if (velPlanet2 > MAX_ANG_VEL)
        {
            angVelPlanet2.mul(MAX_ANG_VEL / velPlanet2);
        }
        if (velPlanet3 > MAX_ANG_VEL)
        {
            angVelPlanet3.mul(MAX_ANG_VEL / velPlanet3);
        }

        // add some damping too
        const double DAMPING = 0.1;
        angVel.mul(1.0 - DAMPING * timeInterval);
        angVelPlanet2.mul(1.0 - DAMPING * timeInterval);
        angVelPlanet3.mul(1.0 - DAMPING * timeInterval);

        linVel.mul(1.0 - DAMPING * timeInterval);
        linVelPlanet2.mul(1.0 - DAMPING * timeInterval);
        linVelPlanet3.mul(1.0 - DAMPING * timeInterval);

        // if user switch is pressed, set velocity to zero
        /*if (tool->getUserSwitch(0) == 1)
        {
            angVel.zero();
            angVelPlanet2.zero();
            angVelPlanet3.zero();

            linVel.zero();
            linVelPlanet2.zero();
            linVelPlanet3.zero();
        }
        */

        // compute the next rotation configuration of the object
        if (angVel.length() > C_SMALL)
        {
            object->rotateAboutGlobalAxisRad(cNormalize(angVel), timeInterval * angVel.length());
        }
        if (angVelPlanet2.length() > C_SMALL)
        {
            object_2->rotateAboutGlobalAxisRad(cNormalize(angVelPlanet2), timeInterval * angVelPlanet2.length());
        }
        if (angVelPlanet3.length() > C_SMALL)
        {
            object_3->rotateAboutGlobalAxisRad(cNormalize(angVelPlanet3), timeInterval * angVelPlanet3.length());
        }



        // compute the next position of the object and cursor
        if (objectPos.length() < 3){
            cVector3d nextPos = object->getLocalPos() + (timeInterval * linVel);
            object->setLocalPos(nextPos);
            m_cursor->setLocalPos(toolPos);
        } else if (objectPos.length() > 2){
            linVel = linVel.length()*cNegate(objectPos)*0.2;
            cVector3d nextPos = object->getLocalPos() + (timeInterval * linVel);
            object->setLocalPos(nextPos);
        }

        if (object_2Pos.length() < 3){
            cVector3d nextPos_2 = object_2->getLocalPos() + (timeInterval * linVelPlanet2);
            object_2->setLocalPos(nextPos_2);
            m_cursor->setLocalPos(toolPos);
        } else if (object_2Pos.length() > 2){
            linVelPlanet2 = linVelPlanet2.length()*cNegate(object_2Pos)*0.2;
            cVector3d nextPos_2 = object_2->getLocalPos() + (timeInterval * linVelPlanet2);
            object_2->setLocalPos(nextPos_2);
        }

        if (object_3Pos.length() < 3){
            cVector3d nextPos_3 = object_3->getLocalPos() + (timeInterval * linVelPlanet3);
            object_3->setLocalPos(nextPos_3);
            m_cursor->setLocalPos(toolPos);
        } else if (object_3Pos.length() > 2){
            linVelPlanet3 = linVelPlanet3.length()*cNegate(object_3Pos)*0.2;
            cVector3d nextPos_3 = object_3->getLocalPos() + (timeInterval * linVelPlanet3);
            object_3->setLocalPos(nextPos_3);
        }


        //cVector3d nextPos = object->getLocalPos() + (timeInterval * linVel);
        //cVector3d nextPos_2 = object_2->getLocalPos() + (timeInterval * linVelPlanet2);
        //cVector3d nextPos_3 = object_3->getLocalPos() + (timeInterval * linVelPlanet3);

        //object->setLocalPos(nextPos);

        //object->setLocalPos(0,0,0);

        //object_2->setLocalPos(nextPos_2);
        //object_3->setLocalPos(nextPos_3);

        //m_debugLabel4->setLocalPos((nextPos.y()*100.0f+500),(nextPos.z()*100.0f+500));

        // added
        // ###########semifunctional#############################################
        //m_cursor->setLocalPos(toolPos);


        //at the surface of the object, turn of haptics due to weird vibrations
        // 0.8 is object radius (approx)

        /*
        if (newtonsGravitationalForceMagnitude < 10) {
            // do nothing
            currentForce = newtonsGravitationalForceMagnitude;
        } else {
            gravityForceM_cursorEarth = newtonsGravitationalForceDirection*newtonsGravitationalForceMagnitude; //cNegate(10*newtonsGravitationalForceDirection);
            //gravityForceM_cursorEarth = cVector3d(0,0,0);
            currentForce = 10;
        }
        */

        //float earthRadius = 0.05;

        /*
        if (newtonsGravitationalForceMagnitude > 10) {
            newtonsGravitationalForceMagnitude = 10;
            gravityForceM_cursorEarth = cNegate(newtonsGravitationalForceDirection)*newtonsGravitationalForceMagnitude;
        }
        */
        /*
        if (newtonsGravitationalForceMagnitudePlanet2 < 10) {
            // do nothing
            currentForce_2 = newtonsGravitationalForceMagnitudePlanet2;
        } else {
            gravityForceM_cursorPlanet2 = cNegate(10*newtonsGravitationalForceDirectionPlanet2); // WRONG
            currentForce_2 = 10;
        }

        if (newtonsGravitationalForceMagnitudePlanet3 < 10) {
            // do nothing
            currentForce_3 = newtonsGravitationalForceMagnitudePlanet3;
        } else {

            gravityForceM_cursorPlanet3 = cNegate(10*newtonsGravitationalForceDirectionPlanet3);
            currentForce_3 = 10;
        }
        */



        // Task 1 w/ haptics
        if (useHaptics_object == true && useHaptics_object_2 == false && useHaptics_object_3 == false){
           //cVector3d totalForce =
           gravityForceM_cursorPlanet2 = cVector3d(0,0,0);
           gravityForceM_cursorPlanet3 = cVector3d(0,0,0);
           hapticDevice->setForce(gravityForceM_cursorEarth+gravityForceM_cursorPlanet2+gravityForceM_cursorPlanet3);
           //hapticDevice->setForce(gravityForceM_cursorPlanet3);

        }
        // Task 2 w/ haptics
        else if (useHaptics_object == false && useHaptics_object_2 == true && useHaptics_object_3 == true){
            gravityForceM_cursorEarth = cVector3d(0,0,0);
            hapticDevice->setForce(gravityForceM_cursorEarth+gravityForceM_cursorPlanet2+gravityForceM_cursorPlanet3);
        }
        // Any task w/o haptics
        else if (useHaptics_object == false && useHaptics_object_2 == false && useHaptics_object_3 == false){
            noForce = cVector3d(0,0,0);
            hapticDevice->setForce(noForce);
        }

        if (task1 == true && startScreen == false){
            /////////////////////////////////////////////////////////////////////
            // Title label
            std::stringstream titleLabel;
            titleLabel << "Task 1" << endl;
            m_titleLabel->setText(titleLabel.str());
            m_titleLabel->setLocalPos((int)(0.5 * (windowW - m_titleLabel->getWidth())), windowH*0.9);

            /////////////////////////////////////////////////////////////////////
            // debug label 2
            std::stringstream ss2;
            float print2 = roundf(v.length() * 100)-1;
            ss2 << "Distance:    " << print2 << endl;
            m_debugLabel2->setText(ss2.str());
            m_debugLabel2->setFontScale(1.3);
            m_debugLabel2->m_fontColor.setRed();

            m_debugLabel2->setLocalPos(0.2*windowW, windowH*0.5);


            /////////////////////////////////////////////////////////////////////
            // label 3
            std::stringstream ss3;
            float print3 = roundf(gravityForceM_cursorEarth.length() * 100) ;
            ss3 << "Force:           " << print3 << endl;
            m_debugLabel3->setText(ss3.str());
            m_debugLabel3->setFontScale(1.3);
            m_debugLabel3->m_fontColor.setRed();

            m_debugLabel3->setLocalPos(0.2*windowW, windowH*0.5-m_debugLabel3->getTextHeight());

            /////////////////////////////////////////////////////////////////////
            // label 4
            std::stringstream ss4;
            ss4 << "" << endl;
            m_debugLabel4->setText(ss4.str());
            m_debugLabel4->setLocalPos(30, 90, 0);

            /////////////////////////////////////////////////////////////////////
            // label 5
            std::stringstream ss5;
            ss5 << "" << endl;
            m_debugLabel5->setText(ss5.str());
            m_debugLabel5->setLocalPos(30, 70, 0);

            /////////////////////////////////////////////////////////////////////
            // label 8
            std::stringstream ss8;
            float print8 = roundf(newtonsGravitationalForceMagnitudePlanet2 * 100) ;
            ss8 << "" << endl;
            m_debugLabel8->setText(ss8.str());
            m_debugLabel8->setFontScale(1.3);
            m_debugLabel8->m_fontColor.setRed();

            m_debugLabel8->setLocalPos(0.2*windowW, windowH*0.7);
        } else if (task1 == false && startScreen == false){
            /////////////////////////////////////////////////////////////////////
            // Title label
            std::stringstream titleLabel;
            titleLabel << "Task 2" << endl;
            m_titleLabel->setText(titleLabel.str());
            m_titleLabel->setLocalPos((int)(0.5 * (windowW - m_titleLabel->getWidth())), windowH - m_titleLabel->getTextHeight());

            /////////////////////////////////////////////////////////////////////
            // label 2
            std::stringstream ss2;
            float print2 = roundf(v_2.length() * 100)-1;
            ss2 << "Distance:    " << print2 << endl;
            m_debugLabel2->setText(ss2.str());
            m_debugLabel2->setFontScale(1.3);
            m_debugLabel2->m_fontColor.setRed();

            m_debugLabel2->setLocalPos(0.2*windowW, windowH*0.6);


            /////////////////////////////////////////////////////////////////////
            // label 8
            std::stringstream ss8;
            float print8 = roundf(newtonsGravitationalForceMagnitudePlanet2UNTOUCHED * 100) ;
            ss8 << "Force:      " << print8 << endl;
            m_debugLabel8->setText(ss8.str());
            m_debugLabel8->setFontScale(1.3);
            m_debugLabel8->m_fontColor.setRed();

            m_debugLabel8->setLocalPos(0.2*windowW, windowH*0.6-m_debugLabel8->getTextHeight());

            /////////////////////////////////////////////////////////////////////
            // label 4
            std::stringstream ss4;
            float print4 = roundf(v_3.length() * 100)-1;
            ss4 << "Distance:    " << print4 << endl;
            m_debugLabel4->setText(ss4.str());
            m_debugLabel4->setFontScale(1.3);
            m_debugLabel4->m_fontColor.setGreen();

            m_debugLabel4->setLocalPos(0.2*windowW, windowH*0.3);


            /////////////////////////////////////////////////////////////////////
            // label 5
            std::stringstream ss5;
            float print5 = roundf(newtonsGravitationalForceMagnitudePlanet3UNTOUCHED * 100) ;
            ss5 << "Force:      " << print5 << endl;
            m_debugLabel5->setText(ss5.str());
            m_debugLabel5->setFontScale(1.3);
            m_debugLabel5->m_fontColor.setGreen();

            m_debugLabel5->setLocalPos(0.2*windowW, windowH*0.3-m_debugLabel5->getTextHeight());

            /////////////////////////////////////////////////////////////////////
            // label 3
            std::stringstream ss3;
            float print3 = roundf(gravityForceM_cursorPlanet3.length() * 100) ;
            ss3 << "" << endl;
            m_debugLabel3->setText(ss3.str());
            m_debugLabel3->setFontScale(1.3);
            m_debugLabel3->m_fontColor.setGreen();

            m_debugLabel3->setLocalPos(0.2*windowW, windowH*0.2-m_debugLabel3->getTextHeight());
        }

    } // end of simulationRunning loop

    //tool->applyToDevice();
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
