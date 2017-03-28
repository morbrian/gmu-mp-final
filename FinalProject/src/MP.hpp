#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#include "Simulator.hpp"

#include <vector>
using std::vector;

#include <iostream>
using std::ostream;
using std::istream;

//
// container for path steps and current path position
// each step of m_way is an index to a Vertex in vertex tree
//
struct Path
{
    vector<int> m_way;
    int              m_pathPos;
};

//
// Vertex represents valid state in the sampled tree
//
struct Vertex
{
    enum
	{
	    TYPE_NONE = 0,
	    TYPE_INIT = 1,
	    TYPE_GOAL = 2
	};
	
    int    m_parent;
    State m_state;
    int    m_type;
    int    m_nchildren;
    int    m_tstep;
    
};

//
// \class MotionPlanner
// Base class for any sampling based planner.
//
class MotionPlanner
{
    friend ostream& operator<<(ostream& stream, const MotionPlanner& mp);
    friend istream& operator>>(istream& stream, MotionPlanner& mp);
    friend class Graphics;
    friend class PriorityFunction;
public:
    //
    // Construct MotionPlanner
    // @param simulator maintains reference to simulator
    // @param id unique ID of the motion planner
    // @param robot planning robog (MotionPlanner will delete)
    // 
    MotionPlanner(Simulator * const simulator, int id, Robot * robot);
            
    virtual ~MotionPlanner(void);
    
    //
    // @return unique id of this planner
    //
    inline int GetId(void) const {
        return m_mpid;
    }

    //
    // @return true if goal is found
    //
    bool IsProblemSolved(void)
    {
        return m_vidAtGoal >= 0;
    }
    
    //
    // Modify rect with configuratoin at time step t.
    // @param t time step indicator
    // @param rect output of configuration
    //
    void GetBoundingVolume(int t, Rectangle2D& rect);

    //
    // Subclasses should implement algorithm for this.
    //
    virtual void Solve(void) = 0;
        
protected:
    
    //
    // Set the priority of this planner requested priority logic.
    //
    void CalcPriority(PriorityType pType);
    
    //
    // Build a path from the vertices, assumes problem is already solved.
    //
    virtual Path& GetPathFromInitToGoal();

    // 
    // Add valid vertex to tree.
    //
    void AddVertex(Vertex * const v);

    //
    // Extend tree from vertex at index vid to the new state.
    // @param vid index of staring vertex
    // @param sto sampled state to connect
    //
    void ExtendTree(const int    vid, const State sto);
    
    int ExtendTreePathCheck(const int vid, const State sto);
    
    // priority indicator of this planner
    double               m_priority;
    
    // unique id of planner
    int                  m_mpid;
    
    // pointer to simulator
    Simulator            *m_simulator;
    
    // pointer to robot
    Robot                *m_robot;
    
    // sampled connected vertices
    vector<Vertex *> m_vertices;
    
    // index of goal vertex
    int                   m_vidAtGoal;
    
    // time taken to solve
    double                m_totalSolveTime;

    // path from start to goal through vertex tree
    Path m_path;
    
    // this identifies one subclass from another
    // and must be coordinated for new subclasses
    // as it has external meaning in the config files.
    int m_algorithmId;
};

inline ostream& 
operator<<(ostream& stream, const MotionPlanner& mp) {
    stream << mp.m_algorithmId << " ";
    stream << *mp.m_robot;
    return stream;
}

inline istream& 
operator>>(istream& stream, MotionPlanner& mp) {
    stream >> mp.m_priority >> *mp.m_robot;
    return stream;
}

//
// \class RRTPlanner
// Implementation of basic RRT algorithm
//
class RRTPlanner : public MotionPlanner
{
public:
    RRTPlanner(Simulator * const simulator, int id, Robot * robot)
    : MotionPlanner(simulator, id, robot)
    {
        m_algorithmId = 0;
    }
    
    virtual void Solve(void);
};


//
// \class ESTPlaner
// Implementation of EST planner.
//
class ESTPlanner : public MotionPlanner
{
public:
    ESTPlanner(Simulator * const simulator, int id, Robot * robot)
    : MotionPlanner(simulator, id, robot)
    {
        m_algorithmId = 2;
    }
    
    virtual void Solve(void);
};

//
// \class RandomPlanner
// Implementation of Random planner.
//
class RandomPlanner : public MotionPlanner
{
public:
    RandomPlanner(Simulator * const simulator, int id, Robot * robot)
    : MotionPlanner(simulator, id, robot)
    {
        m_algorithmId = 1;
    }
    
    virtual void Solve(void);
};

// function for deciding highest priority
// note that the m_priority is set for each planner in CalcPriority
// and this function simply looks at the calculated result.
class PriorityFunction
{
public:
    PriorityFunction() {}
    
    bool operator() (MotionPlanner*& lhs, MotionPlanner*& rhs) const
    {
        return lhs->m_priority < rhs->m_priority;
    }
};

// Planner demonstrates path smoothing
class SmoothRRTPlanner : public RRTPlanner
{
public:
    SmoothRRTPlanner(Simulator * const simulator, int id, Robot * robot)
    : RRTPlanner(simulator, id, robot)
    {
    }
    
    virtual Path& GetPathFromInitToGoal();
};


#endif
