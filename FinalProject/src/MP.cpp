#include "MP.hpp"
#include "Robot.h"
#include "PseudoRandom.hpp"
#include "MyTimer.hpp"
#include <cstring>

#include <vector>
using std::vector;

#include <iostream>
using std::cout;

#include <cmath>
#include <cfloat>

//
// Create random state (sto) from the goal region with probability = 0.1,
// or from the entire bounding box with probability 1 - p (using SampleState).
//
void select_sto(Simulator& m_simulator,  Robot& robot, State& sto);


MotionPlanner::
MotionPlanner(Simulator * const simulator, int id, Robot* robot)
{
    m_simulator = simulator;
    m_robot = robot;
    m_mpid = id;
    
    Vertex *vinit = new Vertex();
    
    vinit->m_tstep    = 0;
    vinit->m_parent   = -1;   
    vinit->m_nchildren= 0;
    vinit->m_state.center.m_x = m_robot->GetRobotCenter().m_x;
    vinit->m_state.center.m_y = m_robot->GetRobotCenter().m_y;
    vinit->m_state.theta = 0.0;
    
    AddVertex(vinit);
    m_vidAtGoal = -1;
    m_totalSolveTime = 0;
    m_path.m_pathPos = 0;
}

MotionPlanner::
~MotionPlanner(void)
{
    //do not delete m_simulator  
    
    const int n = m_vertices.size();
    for(int i = 0; i < n; ++i)
        delete m_vertices[i];
    
    if (m_robot != NULL)
        delete m_robot;
}

// get the bounding volume at time = t,
// but if  this planner is not done solving
// initial state is returned and caller must
// be wise about how to use that information.
void 
MotionPlanner::
GetBoundingVolume(int t, Rectangle2D& rect)
{
    if (this->IsProblemSolved() && m_path.m_way.size() == 0)
        this->GetPathFromInitToGoal();
    
    // adjust t for use with our calculated path
    if (!this->IsProblemSolved() || t < 0 || m_path.m_way.size() == 0)
        t = 0;
    else if (t >= m_path.m_way.size())
        t = m_path.m_way.size() - 1;
    
    Vertex* v;
    if (m_path.m_way.size() == 0)
        v = m_vertices.at(0);
    else
        v = m_vertices.at(m_path.m_way.at(t));
    configureRectangle(rect, 
                       m_robot->GetRobotWidth(), m_robot->GetRobotLength(), 
                       v->m_state.theta, v->m_state.center);
}

void 
MotionPlanner::
CalcPriority(PriorityType pType)
{
    switch (pType)
    {
        case ORDER_BY_INPUT:
            this->m_priority = - this->m_mpid;
            break;
        case ORDER_BY_REVERSE_INPUT:
            this->m_priority = this->m_mpid;
            break;
        case ORDER_BY_LOWEST_AVERAGE_OBS_DIST:
            // this is a simplistic calculation to robot center,
            // and does not look at closest point on robot
            int n = m_simulator->GetNrObstacles();
            double sum = 0;
            for (int i = 0; i < n; ++i)
            {
                sum += point_distance(m_robot->GetRobotCenter(), 
                                      m_simulator->GetObstacleCenterX(i), 
                                      m_simulator->GetObstacleCenterY(i))
                -  m_simulator->GetObstacleRadius(i);
            }
            this->m_priority = - sum / n;
            break;
    }
}

void 
MotionPlanner::
ExtendTree(const int vid, const State sto)
{
    Vertex* start =  m_vertices.at(vid);
    int tstep = start->m_tstep + 1;
    
    double mag = point_distance(sto.center, start->m_state.center);
    
    // velocity vector from vid to sto
    double unit[] = {
        (sto.center.m_x - start->m_state.center.m_x) / mag,
        (sto.center.m_y - start->m_state.center.m_y) / mag
    };
    double steps = ceil(mag / m_simulator->GetDistOneStep());
    
    Vertex* new_vert;
    State cfg;
    cfg.center.m_x = start->m_state.center.m_x;
    cfg.center.m_y = start->m_state.center.m_y;
    cfg.theta = 0.0;
    
    int parent = m_vertices.size() - 1;
    for (double count = 0; count < steps; ++count) 
    {
        if (count == steps - 1)
        {
            // add the actual sto point on the last iteration,
            // which may be closer than distOneStep
            cfg.center = sto.center;
        }
        else 
        {
            // else step closer to sto by a distance of OneStep.
            cfg.center.m_x += unit[0] * m_simulator->GetDistOneStep();
            cfg.center.m_y += unit[1] * m_simulator->GetDistOneStep();
        }
        m_robot->SetRobotState(cfg.center, cfg.theta);
        if (m_simulator->IsValidState(tstep, m_robot)) 
        {
            new_vert = new Vertex();
            if (count == 0)
                new_vert->m_parent = vid;
            else 
                new_vert->m_parent = ++parent;
            new_vert->m_tstep = tstep++;
            new_vert->m_nchildren = 0;
            new_vert->m_state = cfg;
            if (m_robot->HasRobotReachedGoal())
            {
                new_vert->m_type = Vertex::TYPE_GOAL;
                AddVertex(new_vert);
                return;
            }
            AddVertex(new_vert);
        }
        else 
            break;
    }
}

void 
MotionPlanner::
AddVertex(Vertex * const v)
{
    if(v->m_type == Vertex::TYPE_GOAL)
        m_vidAtGoal = m_vertices.size();
    m_vertices.push_back(v); 
    if(v->m_parent >= 0)
        (++m_vertices[v->m_parent]->m_nchildren);
}

Path& 
MotionPlanner::
GetPathFromInitToGoal()
{
    if (m_path.m_way.size() > 0)
        return m_path;
    
    vector<int> rpath;
    
    rpath.clear();
    
    int i = m_vidAtGoal;
    do
    {
        rpath.push_back(i);
        i = m_vertices[i]->m_parent;	
    } 
    while(i >= 0);
    
    m_path.m_way.clear();
    m_path.m_pathPos = 0;
    for(int i = rpath.size() - 1; i >= 0; --i)
        m_path.m_way.push_back(rpath[i]);
    
    return m_path;
}

// create a random state 0.1 near goal, 1 - 0.1 in entire bb.
void 
select_sto(Simulator& m_simulator, Robot& robot, State& sto)
{
    float chance = float(rand() % 10);
    
    // 9 out of 10 times, choose from the whole box
    if (chance > 0)
    {
        // select from bounding box
        m_simulator.SampleState(sto);
        sto.theta =0.0;
    }
    else 
    {
        double g_radius = robot.GetGoalRadius();
        // select from goal region (within radius distance of center point)
        double r = double(rand());
        double r_part = (double)(rand() % 1000) / 1000;
        sto.center.m_x = robot.GetGoalCenter().m_x + cos(r) * g_radius * r_part * 
        (rand() % 2 == 1 ? 1 : -1);
        sto.center.m_y = robot.GetGoalCenter().m_y + sin(r) * g_radius * r_part * 
        (rand() % 2 == 1 ? 1 : -1);
        sto.theta = 0.0;
    }
}

// RRT Planner
void 
RRTPlanner::
Solve(void)
{
    Clock clk;
    StartTime(&clk);
    // given above
    //your code
    
    State sto;
    select_sto(*m_simulator, *m_robot, sto);
    
    int vid = 0;
    double min_dist = FLT_MAX;
    double test_dist;
    vector<Vertex*>::iterator v = m_vertices.begin();
    vector<Vertex*>::iterator v_end = m_vertices.end();
    for (int id = 0; v != v_end; ++v, ++id)
    {
        test_dist = point_distance(sto.center, (*v)->m_state.center);
        if (test_dist < min_dist)
        {
            min_dist = test_dist;
            vid = id;
        }
    }
    
    ExtendTree(vid, sto);
    
    // given below
    m_totalSolveTime += ElapsedTime(&clk);
}

// Random Planner
void 
RandomPlanner::
Solve(void)
{
    Clock clk;
    StartTime(&clk);

    State sto;
    select_sto(*m_simulator, *m_robot, sto);
    
    int vid = 0;
    
	//randomly pick a vertex from the list of chosen vertices
	int pick = int(rand() % m_vertices.size());
    
	vid = pick;
    
    ExtendTree(vid, sto);
    
    // given below
    m_totalSolveTime += ElapsedTime(&clk);
}

void 
ESTPlanner::
Solve(void)
{
    Clock clk;
    StartTime(&clk);
    
    State sto;
    select_sto(*m_simulator, *m_robot, sto);
    
    int vid = 0;
    
	/*
     Let c_i = number of children of vertex v_i
	 So, as in class, w_i = 1/(1 + c_i * c_i) //vertex weight
	 Let TW  = sum_i (w_i) //sum of weights over all vertices
     
	 r = random(0, TW) //real random number from 0 to TW
     
	 for i = 0 ; i < n; i++) //n is nr. vertices
     tw += 1/(1 + c_i * c_i) //running total of weights up to i
     if (tw >= r)
     vid = i
     break
     */
	double weight = 0;
	double sum = 0;
    
	//compute the sum of the weights
	vector<Vertex*>::iterator v = m_vertices.begin();
    vector<Vertex*>::iterator v_end = m_vertices.end();
	for (int id = 0; v != v_end; ++v, ++id)
    {
		weight = 1 / (1 + ((double)(*v)->m_nchildren * (double)(*v)->m_nchildren));
		sum += weight;
	}
    
	double r = PseudoRandomUniformReal(0, sum);
	double tw = 0;
	
	//reset beginning and ending values
	v = m_vertices.begin();
	v_end = m_vertices.end();
    
	for (int id = 0; v != v_end; ++v, ++id)
    {
		tw += 1 / (1 + ((double)(*v)->m_nchildren * (double)(*v)->m_nchildren));
		if(tw >= r)
		{
			vid = id;
			break;
		}
	}
    
	ExtendTree(vid, sto);
    
    m_totalSolveTime += ElapsedTime(&clk);
}

int 
MotionPlanner::
ExtendTreePathCheck(const int    vid, 
                                       const State sto)
{
    //your code
    
    Vertex* start =  m_vertices.at(vid);
    int tstep = start->m_tstep + 1;
    
    double mag = point_distance(sto.center, start->m_state.center);
    
    // velocity vector from vid to sto
    double unit[] = {
        (sto.center.m_x - start->m_state.center.m_x) / mag,
        (sto.center.m_y - start->m_state.center.m_y) / mag
    };
    double steps = ceil(mag / (m_simulator->GetDistOneStep()));
    
    Vertex* new_vert;
    State cfg;
    cfg.center.m_x = start->m_state.center.m_x;
    cfg.center.m_y = start->m_state.center.m_y;
    cfg.theta = 0.0;
    
    int parent = m_vertices.size() - 1;
	double count = 0;
    for (count = 0; count < steps; ++count) 
    {
		//cout << "HELLO: WEIRD\n";
        if (count == steps - 1)
        {
            // add the actual sto point on the last iteration,
            // which may be closer than distOneStep
            cfg.center = sto.center;
        }
        else 
        {
            // else step closer to sto by a distance of OneStep.
            cfg.center.m_x += unit[0] * (m_simulator->GetDistOneStep());
            cfg.center.m_y += unit[1] * (m_simulator->GetDistOneStep());
        }
        
        m_robot->SetRobotState(cfg.center, cfg.theta);
		if (m_simulator->IsValidState(tstep, m_robot)) 
        {
            new_vert = new Vertex();
            if (count == 0)
                new_vert->m_parent = vid;
            else 
                new_vert->m_parent = ++parent;
            new_vert->m_tstep = tstep++;
            new_vert->m_nchildren = 0;
            new_vert->m_state = cfg;
            if (m_robot->HasRobotReachedGoal())
            {
                new_vert->m_type = Vertex::TYPE_GOAL;
                //AddVertex(new_vert);
				//cout << "Return 2\n";
                return 2;
            }
            //AddVertex(new_vert);
        }
        else 
		{
            break;
		}
    }
	if(count < steps)
	{
		//cout << "Return 1\n";
		return 1;
	}
	else
	{
		//cout << "Return 0\n";
		return 0;
	}
}

Path& 
SmoothRRTPlanner::
GetPathFromInitToGoal(void)
{
	if (m_path.m_way.size() > 0)
        return m_path;
    
    std::vector<int> rpath;
	std::vector<int> spath;
    
    rpath.clear();
	spath.clear();
    
    int i = m_vidAtGoal;
    
	//
	// TODO
	// 1.  Add the intermediate points on the new path found.
	// 2.  May need to create new extend function that doesn't
	//		check if in conflict.
	
	i = m_vidAtGoal;
	do
	{
		//According to algorithm in pdf need to
		//put goal on stack twice
		if(i == m_vidAtGoal)
			rpath.push_back(i);
		rpath.push_back(i);
		i = m_vertices[i]->m_parent;
	}
	while(i >= 0);
	
	m_path.m_way.clear();
    m_path.m_pathPos = 0;
    
	int lastNode = rpath.size() - 1;
	int nextNode = rpath.size() - 2;
	//cout << "RPATH:  " << rpath.size() << "\n";
	m_path.m_way.push_back(rpath[lastNode]);
	for(int i = rpath.size()-3; i >= 0; --i)
	{
		//cout << "lastnode: " << rpath[lastNode] << "\n";
		//cout << "state: x:" << m_vertices[rpath[lastNode]]->m_state.center.m_x << " y:" << 
		//	m_vertices[rpath[lastNode]]->m_state.center.m_y << "\n";
		//cout << "nextnode: " << rpath[nextNode] << "\n";
		//cout << "state: x:" << m_vertices[rpath[nextNode]]->m_state.center.m_x << " y:" << 
		//	m_vertices[rpath[nextNode]]->m_state.center.m_y << "\n";
		
		if(point_distance(m_vertices[rpath[lastNode]]->m_state.center, m_vertices[rpath[nextNode]]->m_state.center) 
           <= m_simulator->GetDistOneStep() || ExtendTreePathCheck(rpath[lastNode],m_vertices[rpath[nextNode]]->m_state) == 0)
		{
			//cout << "Test4\n";
			nextNode = i;
		}
		else
		{
            //cout << "NextNode:  " << nextNode << "\n";
            int start_size = m_vertices.size();
            int check = ExtendTreePathCheck(rpath[lastNode],m_vertices[rpath[nextNode]]->m_state);
            
            if(check == 1) //Vertices checked can't be connected so connect last ones the could
            {
                //Add vertices that will get between the last connectable vertices
                //cout << "Here\n";
                //start_size = m_vertices.size();
                ExtendTree(rpath[lastNode],m_vertices[rpath[nextNode+1]]->m_state);
                //m_path.m_way.push_back(rpath[nextNode + 1]);
                int end_size = m_vertices.size();
                int add_i = start_size;
				
                while (add_i < end_size)
                {
                    m_path.m_way.push_back(add_i);
                    add_i++;
                }
            }
            
            if(check == 2)
            {
                //Add path to Goal Vertex
                ExtendTree(rpath[lastNode],m_vertices[rpath[nextNode]]->m_state);
                int end_size = m_vertices.size();
                int add_i = start_size;
				
                while (add_i < end_size)
                {
                    m_path.m_way.push_back(add_i);
                    add_i++;
                }
            }
            
            
            //spath.push_back(rpath[nextNode]);
            lastNode = nextNode+1;
            nextNode = i;
		}
	}
    
	//cout << "Test2\n";
	//cout << "Value2:  " << m_path.m_way.at(1);
	//m_path.m_way.push_back(rpath[nextNode]);
    
    return m_path;
}









