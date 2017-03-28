/**
 *@file Simulator.hpp
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include "PseudoRandom.hpp"
#include "Robot.h"

class MotionPlanner;

//
// Options for type of priority calculation.
//
typedef     
enum {
    ORDER_BY_INPUT,
    ORDER_BY_REVERSE_INPUT,
    ORDER_BY_LOWEST_AVERAGE_OBS_DIST
} PriorityType;

//
// \class Simulator
// Provides access to basic simulation functions like collision detection
// and maintaining list of multiple planners.
//
class Simulator
{
public:    
    Simulator(void);
    
    ~Simulator(void);
    
    enum
	{
	    STATE_X = 0,
	    STATE_Y = 1,
	    STATE_NR_DIMS = 2
	};
    
    //
    // Read in configuration from file.
    //
    void SetupFromFile(const char fname[]);
    
    // Store current scene to file.
    //
    void StoreToFile(const char fname[]);
    
    // 
    // Get the planner at index i
    // (i should also the unique id)
    //
    MotionPlanner* GetPlanner(int i) const
    {
        return m_planners.at(i);	
    }
    
    //
    // Get number of planners.
    //
    int GetNrPlanners() const
    {
        return m_planners.size();
    }
    
    //
    // Get number of obstacles
    //
    int GetNrObstacles(void) const
    {
        return m_circles.size() / 3;
    }
    
    //
    // Get X coord of obstacle[i]
    //
    double GetObstacleCenterX(const int i) const
    {
        return m_circles[3 * i];
    }
    
    //
    // Get Y coord of obstacle[i]
    //
    double GetObstacleCenterY(const int i) const
    {
        return m_circles[1 + 3 * i];
    }
    
    //
    // Get radius of obstacle[i]
    //
    double GetObstacleRadius(const int i) const
    {
        return m_circles[2 + 3 * i];
    }
    
    //
    // Get distance of a single step.
    //
    double GetDistOneStep(void) const
    {
        return m_distOneStep;
    }
    
    //
    // produce a random configuration into param s.
    //
    void SampleState(State& s) const
    {
        s.center.m_x = PseudoRandomUniformReal(m_bbox[0], m_bbox[2]);
        s.center.m_y = PseudoRandomUniformReal(m_bbox[1], m_bbox[3]);
        s.theta = 0.0;
    }
    
    //
    // @return true if the robot is in a valid state at time = t.
    //
    bool IsValidState(int t, Robot* robot) const;
    
    //
    // Get the scene wide bounding box.
    //
    const double* GetBoundingBox(void) const
    {
        return m_bbox;
    }
    
protected:
    // list of obstacles (all circles)
    std::vector<double> m_circles;
    
    // list of planners (one robot per planner)
    std::vector<MotionPlanner*> m_planners;
    
    // max distance robot travles per step
    double              m_distOneStep;
    
    // scene wide bounding box
    double              m_bbox[4];
    
    // read from input file, used by graphics
    int m_displayType;
    
    // priority strategy for this run
    PriorityType m_priType;
    
    friend class Graphics;
};


#endif
