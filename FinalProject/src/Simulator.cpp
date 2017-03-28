#include "Simulator.hpp"
#include "MP.hpp"
#include <cstring>

#include <iostream>
using std::cout;

#include <vector>
using std::vector;

#include <fstream>
using std::ofstream;
using std::ifstream;
using std::ios;
using std::ios_base;


Simulator::
Simulator(void)
{
    // default priority type, may change during setup from file
    m_distOneStep = 0.2;
    m_displayType = 2;
    m_priType = ORDER_BY_LOWEST_AVERAGE_OBS_DIST;
}

Simulator::
~Simulator(void)
{
    const int n = m_planners.size();
    for(int i = 0; i < n; ++i)
        delete m_planners[i];
}

void 
Simulator::
SetupFromFile(const char fname[])
{
    FILE  *in = fopen(fname, "r");
    char   keyword[100];
    
    if(in)
    {
        while(fscanf(in, "%s", keyword) == 1)
        {
            if(strcmp(keyword, "BBox") == 0)
                fscanf(in, "%lf %lf %lf %lf", 
                       &m_bbox[0], &m_bbox[1], &m_bbox[2], &m_bbox[3]);
            else if(strcmp(keyword, "DistOneStep") == 0)
                fscanf(in, "%lf", &m_distOneStep);
            else if(strcmp(keyword, "PriorityType") == 0)
                fscanf(in, "%i", &m_priType);
            else if(strcmp(keyword, "Robots") == 0)
            {
                int n = 0;
                int p = 0;
                double x, y, w, l, t, gx, gy, gr;
                fscanf(in, "%d", &n);
                Robot* rbot;
                for(int i = 0; i < n; ++i)
                {
                    fscanf(in, "%i %lf %lf %lf %lf %lf %lf %lf %lf", 
                           &p, &x, &y, &w, &l, &t, &gx, &gy, &gr);
                    rbot = new Robot(i, x, y, w, l, t);
                    rbot->SetGoalCenter(gx, gy);
                    rbot->SetGoalRadius(gr);
                    switch (p)
                    {
                        case 0:
                            m_planners.
                            push_back(new RRTPlanner(this, rbot->GetId(), rbot));
                            break;
                        case 1:
                            m_planners.
                            push_back(new RandomPlanner(this, rbot->GetId(), rbot));
                            break;
                        case 2:
                            m_planners.
                            push_back(new ESTPlanner(this, rbot->GetId(), rbot));
                            break;
                        case 3:
                            m_planners.push_back(new SmoothRRTPlanner(this, rbot->GetId(), rbot));
                            break;
                        default:
                            m_planners.
                            push_back(new RRTPlanner(this, rbot->GetId(), rbot));
                            
                    }
                }
            }
            else if(strcmp(keyword, "Obstacles") == 0)
            {
                int n = 0;
                double x, y, r;		
                fscanf(in, "%d", &n);
                for(int i = 0; i < n; ++i)
                {
                    fscanf(in, "%lf %lf %lf", &x, &y, &r);
                    m_circles.push_back(x);
                    m_circles.push_back(y);
                    m_circles.push_back(r);
                }
            }
            else if (strcmp(keyword, "Display") == 0)
            {
                int displayType = 2;
                fscanf(in, "%i", &displayType);
                m_displayType = displayType;
            }
        }
        fclose(in);	    
    }	
    else
        printf("..could not open file <%s>\n", fname);
}

void 
Simulator::
StoreToFile(const char *fname)
{
    ofstream fout(fname, ios::out);
    
    // output bounding box
    fout << "BBox "
    << m_bbox[0] << " "
    << m_bbox[1] << " "
    << m_bbox[2] << " "
    << m_bbox[3] << "\n";
    
    // output dist one step
    fout << "DistOneStep " << m_distOneStep << "\n";
    
    // output Display type
    fout << "Display " << m_displayType << "\n";
    
    // priority type
    fout << "PriorityType " << m_priType << "\n";
    
    // Robots
    fout << "Robots\n" << GetNrPlanners() << "\n";
    for (int i = 0; i < GetNrPlanners(); ++i)
        fout << *m_planners.at(i) << "\n";
    
    fout << "Obstacles\n";
    fout << GetNrObstacles() << "\n";
	for(int i = 0; i < GetNrObstacles(); ++i)
        fout << GetObstacleCenterX(i) << " "
        << GetObstacleCenterY(i) << " "
        << GetObstacleRadius(i) << " "
        << "\n";
    
    fout.close();
}

bool 
Simulator::
IsValidState(int t, Robot* robot) const
{
    Rectangle2D rect = robot->GetBoundingVolume();
    
    // ensure all vertices are in bbox
    Point p;
    for (int v = 0; v < 4; ++v) 
    {
        p = rect.vertices[v];
        if (p.m_x < m_bbox[0] || p.m_x > m_bbox[2] ||
            p.m_y < m_bbox[1] || p.m_y > m_bbox[3])
            return false;
    }
    
    const int    n  = GetNrObstacles();
    
    // if rectangle in conflict with any obstacle, stat is not valid
    for(int i = 0; i < n; ++i)
    {
        Point c = 
        {
            GetObstacleCenterX(i),
            GetObstacleCenterY(i)
        };
        const double r = GetObstacleRadius(i);
        
        if (IsCollisionRectangleCircle(rect, c, r))
            return false;
    }
    
    // if the robot is at the goal, 
    // then we can ignore the other robots in the
    // goal area and this state is valid.
    // if (robot->HasRobotReachedGoal())
    //    return true;
    
    vector<MotionPlanner*>::const_iterator mp = m_planners.begin();
    vector<MotionPlanner*>::const_iterator mp_end = m_planners.end();
    // compare against solved robots
    Rectangle2D otherRect;
    for ( ; mp != mp_end; ++mp)
        if ((*mp)->IsProblemSolved() && ((*mp)->GetId() != robot->GetId()))
        {
            (*mp)->GetBoundingVolume(t, otherRect);
            // great, now check rectangle-rectangle collision
            
            if (IsCollisionRectangleRectangle(rect, otherRect))
                return false;
        }
    return true;
}
