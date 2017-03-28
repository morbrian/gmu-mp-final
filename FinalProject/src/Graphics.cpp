#include "Graphics.hpp"
#include <iostream>
using std::cout;

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif defined _WIN32
#include "glutForWindows.h"
#else
#include <GL/glut.h>
#endif

const long SPEED_FAST = 15;
const long SPEED_HALF = 50;
const long SPEED_TENTH = 100;
const long SPEED_LONG = 1000;
const double ROTATION_STEP = 1.0;

// LIGHTING AND COLOR
const float OBS_SHININESS = 100.0f;
const float LIGHT0_POS[] = {30.0f, 20.0f, 30.0f, 0.0f};
const float LIGHT1_POS[] = {-30.0f, -20.0f, 30.0f, 0.0f};
const float COLOR_BLACK[] = {0.0f, 0.0f, 0.0f, 1.0f};
const float COLOR_BLACK_T[] = {0.0f, 0.0f, 0.0f, 0.3f};
const float COLOR_BLACKISH[] = {0.3f, 0.3f, 0.3f, 1.0f};
const float COLOR_WHITE[] = {1.0f, 1.0f, 1.0f, 1.0f};
const float COLOR_WHITE_T[] = {1.0f, 1.0f, 1.0f, 0.1f};
const float COLOR_WHITISH[] = {0.8f, 0.8f, 0.8f, 1.0f};
const float COLOR_WHITISH_T[] = {0.8f, 0.8f, 0.8f, 0.8f};
const float COLOR_RED[] = {1.0f, 0.0f, 0.0f, 1.0f};
const float COLOR_RED_T[] = {1.0f, 0.0f, 0.0f, 0.3f};
const float COLOR_REDISH[] = {0.3f, 0.0f, 0.0f, 1.0f};
const float COLOR_GREEN[] = {0.0f, 1.0f, 0.0f, 1.0f};
const float COLOR_GREEN_T[] = {0.0f, 1.0f, 0.0f, 0.3f};
const float COLOR_GREENISH[] = {0.0f, 0.3f, 0.0f, 1.0f};
const float COLOR_BLUE[] = {0.0f, 0.0f, 1.0f, 1.0f};
const float COLOR_BLUE_T[] = {0.0f, 0.0f, 1.0f, 0.3f};
const float COLOR_BLUEISH[] = {0.0f, 0.0f, 0.3f, 1.0f};
const float COLOR_COLOR_MAGENTA[] = {1.0f, 0.0f, 1.0f, 1.0f};
const float COLOR_MAGENTA_T[] = {1.0f, 0.0f, 1.0f, 0.3f};
const float COLOR_MAGENTISH[] = {0.7f, 0.0f, 0.7f, 1.0f};
const float COLOR_CYAN[] = {0.0f, 1.0f, 1.0f, 1.0f};
const float COLOR_CYAN_T[] = {0.0f, 1.0f, 1.0f, 0.3f};
const float COLOR_CYANISH[] = {0.0f, 0.7f, 0.7f, 1.0f};
const float COLOR_YELLOW[] = {1.0f, 1.0f, 0.0f, 1.0f};
const float COLOR_YELLOW_T[] = {1.0f, 1.0f, 0.0f, 0.3f};
const float COLOR_YELLISH[] = {0.7f, 0.7f, 0.0f, 1.0f};

Graphics *m_graphics = NULL;

bool m_use3D = false;

long TIMER_DELAY = SPEED_FAST;

Graphics::
Graphics(const char fname[]) 
{
    m_simulator.SetupFromFile(fname);
    
    m_use3D = m_simulator.m_displayType == 3;
    
    PlannerHeap tempHeap;
    // build the priority heap using configure priority logic
    for (int rid = 0; rid < m_simulator.GetNrPlanners(); ++rid)
    {
        MotionPlanner* mp = m_simulator.GetPlanner(rid);
        mp->CalcPriority(m_simulator.m_priType);
        m_plannerHeap.push(mp);
        tempHeap.push(mp);
    }
    
    // show our priority function by name
    cout << "Priority Function: " 
              << PRIORITY_NAMES[m_simulator.m_priType % END_PRIORITY] << "\n";
    
    // use the temp heap to print out priority order
    cout << "Robot Planning Order:\n";
    while (!tempHeap.empty())
    {
        MotionPlanner* mp = tempHeap.top();
        cout << "rbot-" << mp->GetId() << ": " 
        << COLOR_NAMES[mp->GetId() % END_COLORS] << "\n";
        tempHeap.pop();
    }
    
    m_selectedGoal   = -1;
    m_selectedRobot  = -1;
    m_selectedCircle = -1;
    m_editRadius     = false;
    m_run            = false;
    m_stepOnce       = false;
    m_printRobotCorners = false;
    
    m_drawPlannerVertices = true;
    
    m_delay = 1;
    
    m_xRot = 0.0;
    m_yRot = 0.0;
    m_zRot = 0.0;
}

Graphics::~Graphics(void)
{
    //
}

void 
Graphics::
MainLoop(void)
{	
    m_graphics = this;
    
    //create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(1000, 600);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Planner");	   	
    
    
    //register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutMotionFunc(CallbackEventOnMouseMotion);
    glutIdleFunc(NULL);
    glutTimerFunc(1, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);
    
    //enter main event loop
    glutMainLoop();	
}

// Request planning of a single planner
void 
Graphics::
ProcessPlanner(MotionPlanner* planner)
{
    int pid = planner->GetId();
    // if continuous run, or single step then process
    if((m_run || m_stepOnce))
    {
        // request 1000 sample points in search for goal
        for(int i = 0; i < 1000 && !planner->IsProblemSolved(); ++i)
        {
            planner->Solve();
        }
        if(!planner->IsProblemSolved())
            cout << "(id=" << pid 
            << ") TotalSolveTime = " << planner->m_totalSolveTime
            << "[Solved = " << planner->IsProblemSolved()
            << "] [NrVertices = " << planner->m_vertices.size()
            << "]\n";
    }    
}

// Animate the path traversal of planner.
void 
Graphics::
AnimatePath(MotionPlanner* planner, bool& arrived)
{
    // if problem solved, animate the path
    if(planner->IsProblemSolved())
    {
        Path& path = planner->GetPathFromInitToGoal();
        
        // if we are at last path index, we are at goal
        arrived = path.m_way.size() != 0 && path.m_pathPos >= path.m_way.size();
        
        // if we have a valid path position
        if(path.m_way.size() != 0 && path.m_pathPos < path.m_way.size())
        {
            // put robot in state for the given path location
            planner->m_robot->
                SetRobotState(planner->
                              m_vertices[path.m_way[path.m_pathPos]]->m_state);
            
            // print debugging info if enabled
            if (m_printRobotCorners)
            {
                cout << "id-" << planner->GetId() << " at t = " 
                << path.m_pathPos 
                << ", thinks its: " 
                << (planner->m_vertices[path.m_way[path.m_pathPos]]->m_tstep) 
                << "\n";
                planner->m_robot->GetBoundingVolume().printCorners();
                cout << "\n";
            }
            
            // increment path location for next animation frame
            ++path.m_pathPos;
        }
    }
}

void 
Graphics::
HandleEventOnTimer(void)
{
    if (!m_run && !m_stepOnce)
        return;
    
    // if no unsolved planners, then animate paths
    if (m_plannerHeap.empty())
    {
        int arrivedCount = 0;
        bool arrived;
        
        int n = m_simulator.GetNrPlanners();
        for (int pid = 0; pid < n; ++pid)
        {
            arrived = false;
            AnimatePath(m_simulator.GetPlanner(pid), arrived);
            if (arrived)
                ++arrivedCount;
        }
        
        // if all animations arrived at goal,
        // reset the animation
        if(arrivedCount == n)
            for (int pid = 0; pid < n; ++pid)
                m_simulator.GetPlanner(pid)->m_path.m_pathPos = 0;
    }
    else 
    {
        MotionPlanner* mp = m_plannerHeap.top();
        ProcessPlanner(mp);
        if (mp->IsProblemSolved())
        {
            m_plannerHeap.pop();
        }
        
        // print summary results after all robots complete.
        if (m_plannerHeap.empty())
        {
            int n = m_simulator.GetNrPlanners();
            for (int pid = 0; pid < n; ++pid)
            {
                MotionPlanner* planner = m_simulator.GetPlanner(pid);
                cout << "(id=" << pid
                << ") TotalSolveTime = " << planner->m_totalSolveTime
                << " [Solved = " << planner->IsProblemSolved()
                << "] [NrVertices = " <<  planner->m_vertices.size()
                << "]\n";
            }
        }
    }
    m_stepOnce = false;
}

void 
Graphics::
HandleEventOnMouseMotion(const double mousePosX, const double mousePosY)
{
    // this was saying after planning starts, don't permit mouse motion events
    if(m_run && m_plannerHeap.empty())
        return;
    
    if(m_selectedCircle >= 0)
    {
        if(m_editRadius)
        {
            const double cx = m_simulator.m_circles[3 * m_selectedCircle];
            const double cy = m_simulator.m_circles[3 * m_selectedCircle + 1];
            
            m_simulator.m_circles[3 * m_selectedCircle + 2] = 
            sqrt((cx - mousePosX) * (cx - mousePosX) +
                 (cy - mousePosY) * (cy - mousePosY));
        }
        else
        {
            m_simulator.m_circles[3 * m_selectedCircle] = mousePosX;
            m_simulator.m_circles[3 * m_selectedCircle + 1] = mousePosY;
        }
        
    }
    else if (m_selectedRobot >= 0)
    {
        MotionPlanner* mp = m_simulator.m_planners.at(m_selectedRobot);
        Point c =
        {
            mousePosX,
            mousePosY
        };
        mp->m_vertices.at(0)->m_state.center = c;
        mp->m_robot->SetRobotCenter(c);
        mp->m_robot->commit();
    }
    else if (m_selectedGoal >= 0)
    {
        MotionPlanner* mp = m_simulator.m_planners.at(m_selectedGoal);
        mp->m_robot->SetGoalCenter(mousePosX, mousePosY);
        
    }
}

void 
Graphics::
HandleEventOnMouseBtnDown
(const int whichBtn, const double mousePosX, const double mousePosY)
{   
    // this was saying after planning starts, don't permit mouse motion events
    if(m_run && m_plannerHeap.empty())
        return;
    m_selectedCircle = -1;
    m_selectedRobot = -1;
    m_selectedGoal = -1;
    
    Point clickPoint =
    {
        mousePosX,
        mousePosY
    };
    MotionPlanner* mp;
    Rectangle2D rect;
    for (int i = 0; i < m_simulator.m_planners.size(); ++i)
    {
        mp = m_simulator.m_planners.at(i);
        mp->GetBoundingVolume(0, rect);
        if (rect.inBounds(&clickPoint))
        {
            m_selectedRobot = i;
            return;
        }
        
        const double d = 
        point_distance(mp->m_robot->m_goal, mousePosX, mousePosY);
        if (d <= mp->m_robot->m_goalRadius)
        {
            m_selectedGoal = i;
            return;
        }
    }
    
    // was an obstacle clicked on?
    for(int i = 0; 
        i < m_simulator.m_circles.size() && m_selectedCircle == -1; 
        i += 3)
    {
        const double cx = m_simulator.m_circles[i];
        const double cy = m_simulator.m_circles[i + 1];
        const double r  = m_simulator.m_circles[i + 2];
        const double d  = 
        sqrt((mousePosX - cx) * (mousePosX - cx) 
             + (mousePosY - cy) * (mousePosY - cy));
        
        if(d <= r)
            m_selectedCircle = i / 3;
    }
    
    // if neither robots nor obstacles clicked, then create new obstacle
    if(m_selectedCircle == -1)
    {
        m_simulator.m_circles.push_back(mousePosX);
        m_simulator.m_circles.push_back(mousePosY);
        m_simulator.m_circles.push_back(1.0);
    }    
}

void 
Graphics::
HandleEventOnKeyPress(const int key)
{
    switch(key)
    {
        case 27: //escape key
            exit(0);
            
        case 'r':
            m_editRadius = !m_editRadius;	
            break;
            
        case 'p':
            m_run = !m_run;
            cout << "ALLOW RUNNING = " << m_run << "\n";
            break;
            
        case 'v':
            m_drawPlannerVertices = !m_drawPlannerVertices;
            break;
            
        case 'd':
            m_printRobotCorners = true;
            break;
            
        case 't':
            m_stepOnce = true;
            cout << "+";
            break;
            
        case 'z':
            m_zRot += ROTATION_STEP;
            break;
            
        case 'Z':
            m_zRot -= ROTATION_STEP;
            break;
            
        case 'x':
            m_xRot += ROTATION_STEP;
            break;
            
        case 'X':
            m_xRot -= ROTATION_STEP;
            break;
            
        case 'y':
            m_yRot += ROTATION_STEP;
            break;
            
        case 'Y':
            m_yRot -= ROTATION_STEP;
            break;
            
        case '1': 
            TIMER_DELAY = SPEED_FAST;
            break;
            
        case '2':
            TIMER_DELAY = SPEED_HALF;
            break;
        case '3':
            TIMER_DELAY = SPEED_TENTH;
            break;
            
        case '4':
            TIMER_DELAY = SPEED_LONG;
            break;
            
        case 's':
            m_simulator.StoreToFile("scene.txt");
    }
}

void 
Graphics::
HandleEventOnDisplay(void)
{
    
    glPushMatrix();
    glRotated(m_xRot, 1.0, 0.0, 0.0);
    glRotated(m_yRot, 0.0, 1.0, 0.0);
    glRotated(m_zRot, 0.0, 0.0, 1.0);
    
    //draw bounding box
    const double *bbox = m_graphics->m_simulator.m_bbox;
    
    if (m_use3D)
    {
        glMaterialfv(GL_FRONT, GL_AMBIENT, COLOR_WHITISH);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, COLOR_WHITISH);
        glMaterialfv(GL_FRONT, GL_SPECULAR, COLOR_WHITE);
        glMaterialfv(GL_FRONT, GL_EMISSION, COLOR_BLACK);
        glMaterialfv(GL_FRONT, GL_SHININESS, &OBS_SHININESS);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glBegin(GL_POLYGON);
        glNormal3f(0.,0.,1.);
        glVertex2d(bbox[0], bbox[1]);
        glNormal3f(0.,0.,1.);
        glVertex2d(bbox[2], bbox[1]);
        glNormal3f(0.,0.,1.);
        glVertex2d(bbox[2], bbox[3]);
        glNormal3f(0.,0.,1.);
        glVertex2d(bbox[0], bbox[3]);
        glEnd();
    }
    else
    {
        glColor3f(0, 0, 1);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBegin(GL_POLYGON);
        glVertex2d(bbox[0], bbox[1]);
        glVertex2d(bbox[2], bbox[1]);
        glVertex2d(bbox[2], bbox[3]);
        glVertex2d(bbox[0], bbox[3]);
        glEnd();
        
    }
    
    // draw obstacles
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    if (m_use3D)
    {
        glMaterialfv(GL_FRONT, GL_AMBIENT, COLOR_BLUE);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, COLOR_BLUE);
        glMaterialfv(GL_FRONT, GL_SPECULAR, COLOR_WHITE);
        glMaterialfv(GL_FRONT, GL_EMISSION, COLOR_BLACK);
        glMaterialfv(GL_FRONT, GL_SHININESS, &OBS_SHININESS);
        for(int i = 0; i < m_simulator.GetNrObstacles(); ++i)
            DrawCircle3D(m_simulator.GetObstacleCenterX(i),
                         m_simulator.GetObstacleCenterY(i),
                         m_simulator.GetObstacleRadius(i));
    }
    else 
    {
        glColor3f(0, 0, 1);
        for(int i = 0; i < m_simulator.GetNrObstacles(); ++i)
            DrawCircle2D(m_simulator.GetObstacleCenterX(i),
                         m_simulator.GetObstacleCenterY(i),
                         m_simulator.GetObstacleRadius(i));
        
    }
    
    
    // Draw all planner content (robot, goals, vertices, paths)
    
	glPointSize(4.0);
	
    MotionPlanner* planner;
    int n = m_simulator.GetNrPlanners();
    for (int pid = 0; pid < n; ++pid)
    {
        planner = m_simulator.GetPlanner(pid);
        
        //draw robots and goals
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        Robot* rbot = planner->m_robot;
        if (m_use3D)
        {
            glMaterialfv(GL_FRONT, GL_AMBIENT, COLORS[pid % END_COLORS]);
            glMaterialfv(GL_FRONT, GL_DIFFUSE, COLORS[pid % END_COLORS]);
            glMaterialfv(GL_FRONT, GL_SPECULAR, COLOR_WHITE);
            glMaterialfv(GL_FRONT, GL_EMISSION, COLOR_BLACK);
            glMaterialfv(GL_FRONT, GL_SHININESS, &OBS_SHININESS);
            DrawCircle3D(rbot->GetGoalCenter().m_x, 
                         rbot->GetGoalCenter().m_y, 
                         rbot->GetGoalRadius());
            DrawRobot3D(*rbot);
        }
        else
        {
            glColor4fv(COLORS[pid % END_COLORS]);
            DrawCircle2D(rbot->GetGoalCenter().m_x, 
                         rbot->GetGoalCenter().m_y, 
                         rbot->GetGoalRadius());
            DrawRectangle2D(rbot->GetBoundingVolume());
            
        }
        
        //path taken as planned to current position
        if (planner->IsProblemSolved())
        {
            int end = planner->m_path.m_pathPos;
            glBegin(GL_LINES);
            for (int wid = 1; wid < end; ++wid)
            {
                Vertex* v = 
                planner->m_vertices.at(planner->m_path.m_way.at(wid - 1));
                glVertex3d(v->m_state.center.m_x, v->m_state.center.m_y, 0.2);
                v = planner->m_vertices.at(planner->m_path.m_way.at(wid));
                glVertex3d(v->m_state.center.m_x, v->m_state.center.m_y, 0.2);
                
            }
            glEnd();
        }
        
        //draw planner vertices
        if(m_drawPlannerVertices)
        {
            const int n = planner->m_vertices.size();
            //glColor3f(0.6, 0.8, 0.3);	
            glBegin(GL_POINTS);	
            for(int i = 0; i < n; ++i)
                glVertex3d(planner->m_vertices[i]->m_state.center.m_x, 
                           planner->m_vertices[i]->m_state.center.m_y, 0.1);
            glEnd();
            glBegin(GL_LINES);	
            for(int i = 1; i < n; ++i)
            {
                glVertex3d(planner->m_vertices[i]->m_state.center.m_x, 
                           planner->m_vertices[i]->m_state.center.m_y, 0.1);
                glVertex3d(planner->m_vertices[planner->m_vertices[i]->m_parent]->m_state.center.m_x,
                           planner->m_vertices[planner->m_vertices[i]->m_parent]->m_state.center.m_y, 0.1);
            }
            glEnd();
        }
    }
    glPopMatrix();
}

void 
Graphics::
DrawRectangle2D(const Rectangle2D& rect)
{
    glBegin(GL_POLYGON);
    //Point* end = rect.vertices + 4;
    for (int i = 0; i < 4; ++i)
        glVertex2d(rect.vertices[i].m_x, rect.vertices[i].m_y);
    glEnd();
}

void 
Graphics::
DrawRobot3D(const Robot& robot)
{
    glPushMatrix();
    glTranslated(robot.GetRobotCenter().m_x, robot.GetRobotCenter().m_y, 0.0);
    glRotated(robot.GetRobotOrientation(), 0, 1, 0);
    glScaled(robot.GetRobotWidth(), robot.GetRobotLength(), robot.GetRobotWidth());
    glutSolidCube(1);
    glPopMatrix();
    
}


void 
Graphics::
DrawCircle2D(const double cx, const double cy, const double r)
{
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
        glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
    glEnd();	
}

void 
Graphics::
DrawCircle3D(const double cx, const double cy, const double r)
{
    glPushMatrix();
    glTranslated(cx, cy, 0);
    glutSolidSphere(r, 32, 32);
    glPopMatrix();
}

void 
Graphics::
CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
        if (m_use3D)
        {
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
            glEnable(GL_LIGHTING);
            glEnable(GL_NORMALIZE);
            
            // LIGHT0
            glEnable(GL_LIGHT0);
            glLightfv(GL_LIGHT0, GL_POSITION, LIGHT0_POS);
            glLightfv(GL_LIGHT0, GL_DIFFUSE, COLOR_WHITE);
            glLightfv(GL_LIGHT0, GL_SPECULAR, COLOR_WHITE);
            
            // LIGHT1
            glEnable(GL_LIGHT1);
            glLightfv(GL_LIGHT1, GL_POSITION, LIGHT1_POS);
            glLightfv(GL_LIGHT1, GL_DIFFUSE, COLOR_WHITISH);
            glLightfv(GL_LIGHT1, GL_SPECULAR, COLOR_WHITISH);
            
            glClearDepth(1.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
            glEnable(GL_DEPTH_TEST);
            glShadeModel(GL_SMOOTH);	
            
            glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
            
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            
            const double *bbox = m_graphics->m_simulator.m_bbox;
            
            
            //glOrtho(bbox[0] - 1, bbox[2] + 1, bbox[1] - 1, bbox[3] + 1, -1.0, 1.0);
            float w = bbox[2] - bbox[0] + 1;
            float h = bbox[3] - bbox[1] + 1;
            MyPerspective(40.0f, w / h, w / 2, 4.0 * w);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            
            glTranslatef(0.0f, 0.0f, -1.2f * w);
            glRotated(-45, 1.0, 0.0, 0.0);
        }
        else 
        {
            glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
            glClearDepth(1.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
            glEnable(GL_DEPTH_TEST);
            glShadeModel(GL_SMOOTH);	
            
            glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
            
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            
            const double *bbox = m_graphics->m_simulator.m_bbox;
            
            glOrtho(bbox[0] - 1, bbox[2] + 1, bbox[1] - 1, bbox[3] + 1, -1.0, 1.0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
        }
        
        m_graphics->HandleEventOnDisplay();
        
        glutSwapBuffers();	    
    }
}

// Set a 3D perspective
// Reference: GMU CS652 - Foundations of 3D Graphics Programming, Chen
void 
Graphics::
MyPerspective(float fovy, float aspect, float near, float far)
{
    float left, right, bottom, top;
    
    fovy = fovy * M_PI / 180.0f;
    
    top = near * tan(fovy / 2.0f);
    bottom = - top;
    right = aspect * top;
    left = - right;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(left, right, bottom , top, near, far);
}


void 
Graphics::
CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics &&  state == GLUT_DOWN)
    {
        double mouseX, mouseY;
        MousePosition(x, y, &mouseX, &mouseY);
        m_graphics->HandleEventOnMouseBtnDown(button, mouseX , mouseY);
        glutPostRedisplay();
    }	    
}

void 
Graphics::
CallbackEventOnMouseMotion(int x, int y)
{
    double mouseX, mouseY;
    MousePosition(x, y, &mouseX, &mouseY);
    m_graphics->HandleEventOnMouseMotion(mouseX , mouseY);
    glutPostRedisplay();
}


void 
Graphics::
CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
        m_graphics->HandleEventOnTimer();
        glutTimerFunc(TIMER_DELAY, CallbackEventOnTimer, id);
        glutPostRedisplay();	    
    }
}

void 
Graphics::
CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
        m_graphics->HandleEventOnKeyPress(key);	
}

void 
Graphics::
MousePosition(const int x, const int y, double *posX, double *posY)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posZ;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview,
                 projection, viewport, posX, posY, &posZ);
}

int 
main(int argc, char **argv)
{
    PseudoRandomSeed();
    
    if(argc < 2)
    {
        printf("missing arguments\n");		
        printf("  Planner <file>\n");
        return 0;		
    }
    
    Graphics graphics(argv[1]);
    
    graphics.MainLoop();
    
    return 0;    
}
