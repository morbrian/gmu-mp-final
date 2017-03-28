/**
 *@file Graphics.hpp
 *@author Erion Plaku 
 *@brief Graphics for running simulation and setting up problem
 */

#ifndef  GRAPHICS_HPP_
#define  GRAPHICS_HPP_

#include "MP.hpp"
#include "Simulator.hpp"
#include "Robot.h"

#include <vector>
using std::vector;

#include <string>
using std::string;

#include <queue>
using std::priority_queue;


// define some colors to differentiate robot/goal pairs
const int END_COLORS = 10;
float COLORS[][4] =
{
    { 1.0f, 0.0f, 0.0f, 1.0f }, // red    
    { 1.0f, 0.6f, 0.0f, 1.0f }, // orange?
    { 1.0f, 0.0f, 1.0f, 1.0f }, // magenta
    { 0.0f, 1.0f, 0.0f, 1.0f }, // green
    { 0.0f, 1.0f, 1.0f, 1.0f }, // cyan?
    { 0.0f, 0.0f, 0.0f, 1.0f }, // black
    { 0.5f, 0.5f, 0.5f, 1.0f }, // grey
    { 0.5f, 0.0f, 1.0f, 1.0f }, // purple?
    { 0.5f, 0.5f, 0.0f, 1.0f }, // mustard?
    { 1.0f, 1.0f, 0.0f, 1.0f } // yellow
};

// define human readable names of robot colors
string COLOR_NAMES[] =
{
    "RED",
    "ORANGE",
    "MAGENTA",
    "GREEN",
    "CYAN",
    "BLACK",
    "GREY",
    "PURPLE",
    "MUSTARD",
    "YELLOW"
};

// define human readable Priority function names
const int END_PRIORITY = 3;
string PRIORITY_NAMES[] =
{
    "Order By Input Sequence",
    "Order By Reverse Input Sequence",
    "Ordre By Smallest Average Obstacle Distance"
};

typedef 
priority_queue<
    MotionPlanner*, 
    vector<MotionPlanner*>, 
    PriorityFunction> 
PlannerHeap;

//
// \class Graphics
// Control graphics display and user interaction.
// Keyboard Options:
// esc: quit
// r: modify obstacle radii
// p: toggle planning start/stop
// v: toggle drawing of vertices
// d: debug output for robot vertices to console
// t: take a single step in planning or animation
// z/Z: rotate positive(z) or negative(Z) on z access
// x/X: rotate positive(x) or negative(X) on x access
// y/Y: rotate positive(y) or negative(Y) on y access
// 1..4: Adjust animation speed (1 = fast, 4 = slowest)
// s: save scene (robots and obstacles) to scene.txt file
//
class Graphics
{   
public:
    
    //
    // Construct graphics scene.
    // @param fname file to read scene config from
    //
    Graphics(const char fname[]);
    
    ~Graphics(void);

    void MainLoop(void);

protected:
    void HandleEventOnTimer(void);
    void HandleEventOnDisplay(void);
    void HandleEventOnMouseBtnDown(const int whichBtn, const double mousePosX, const double mousePosY);
    void HandleEventOnMouseMotion(const double mousePosX, const double mousePosY);
    void HandleEventOnKeyPress(const int key);
    
    //
    // Request planning of a single planner
    //
    void ProcessPlanner(MotionPlanner* planner);
    
    //
    // Animate the path traversal of planner.
    // @param planner planner to animate
    // @param arrived indicate whether animated robot arrived at goal.
    //
    void AnimatePath(MotionPlanner* planner, bool& arrived);

    // Draw 2D Circle
    void DrawCircle2D(const double cx, const double cy, const double r);
    void DrawCircle3D(const double cx, const double cy, const double r);
    void DrawRectangle2D(const Rectangle2D& rect);
    void DrawRobot3D(const Robot& robot);

    static void CallbackEventOnDisplay(void);
    
    // Set a 3D perspective
    // Reference: GMU CS652 - Foundations of 3D Graphics Programming, Chen
    static void MyPerspective(float fovy, float aspect, float near, float far);
    
    static void CallbackEventOnMouse(int button, int state, int x, int y);
    static void CallbackEventOnMouseMotion(int x, int y);
    static void CallbackEventOnTimer(int id);
    static void CallbackEventOnKeyPress(unsigned char key, int x, int y);
    static void MousePosition(const int x, const int y, double *posX, double *posY);
    
    Simulator       m_simulator;

    // this should really be in simulator, but it won't compile there
    // so for now it's here.
    PlannerHeap m_plannerHeap; 

    int   m_selectedGoal;
    int   m_selectedCircle;
    int   m_selectedRobot;
    bool  m_editRadius;
    bool  m_rotateScene;
    bool  m_run;
    
    // draw vertex tree for all planners
    bool m_drawPlannerVertices;
    
    // delay between auto-animate steps
    int m_delay;
    
    // toggles to allow a single step, then togles off
    bool m_stepOnce;
    
    // print debug info about robot corner positions
    bool m_printRobotCorners;
    
    // scene rotation values
    double m_xRot;
    double m_yRot;
    double m_zRot;
    
};

#endif
