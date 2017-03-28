//
//  Robot.h
//  FinalProject
//
// CS689 - Planning Motions of Robots
// Brian Moriarty / Stuart Roetteger
//

#ifndef FinalProject_Robot_h
#define FinalProject_Robot_h
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <iostream>
using std::cout;
using std::ostream;
using std::istream;

// structor for storing single 2D point.
struct Point
{
    double m_x;
    double m_y;
    void printCoords() const
    {
        cout << "{" << m_x << "," << m_y << "}";
    }
};

// representation of current robot state
struct State
{
    Point center;
    double theta;
};

// specific bouding area of 4 vertex rectangle
struct Rectangle2D
{
    Point vertices[4];
    bool inBounds(const Point* p) const;
    void printCorners() const;
};

//
// point_distance
// Calculate distance between 2 points (x1, y1) and (x2, y2)
// @return distance between points
//
double point_distance(const double x1, const double y1, const double x2, const double y2);
double point_distance(const Point& p1, const Point& p2);
double point_distance(const Point& p1, const double x2, const double y2);

//
// ClosestPtPointSegment
// Based on Code from book Realtime Collision Detection by Christer Ericson.
// Chapter 5.
// We use this to find the nearest point on rectangle side to the center point
// of circle obstacles, and then compare that point distance to circle radius.
// There are other ways to accomplish that goal listed in the book, but
// this seems reasonable for a 2D environment.
//
void ClosestPtPointSegment(Point& c, Point& a, Point& b, Point &d);

// Signed2DTriArea
// Based on code from book Realtime Collision Detection by Christer Ericson.
// Chapter 5.
// We use this to find intersecting sides of 2 Rectangle shapes.
double Signed2DTriArea(const Point& a, const Point& b, const Point& c);

// Based on code from book Realtime Collision Detection by Christer Ericson.
// Chapter 5. (see Test2DSegmentSegment)
// @return true if segments intersect
bool IsSegmentIntersection(const Point& a, const Point& b, 
                           const Point& c, const Point& d);

// @return true if rectangle and circle overlap.
bool IsCollisionRectangleCircle(const Rectangle2D& rect, const Point& c, double r);

// @return true if rectangle and rectangle overlap.
bool IsCollisionRectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2);

//
// dot product of 2D vectors a and b
// @param a array of 2 double
// @param b array of 2 double
//
inline double dotprod(const Point a, const Point b);

//
// return true if the point specified by v[] and t1
// are on the same side of the line t2, t3
// Used by Rectangle.
//
inline bool sameSide(const Point& v, const Point& p, const Point& t1, const Point& t2);

//
// configure point based on provided configuration
//
inline void configurePoint(Point& p, double x, double y, double theta, const Point& c)
{
    p.m_x = -x * cos(theta) - y * sin(theta) + c.m_x;
    p.m_y = -x * sin(theta) + y * cos(theta) + c.m_y;
}

//
// configure rectangle based on provided configuration
//
inline void configureRectangle(Rectangle2D& rect, double width, double length, double theta, const Point& c)
{
    double h = length / 2.0;
    double w = width / 2.0;
    // counter clockwise from front left
    configurePoint(rect.vertices[0], -w, h, theta, c);
    configurePoint(rect.vertices[1], w, h, theta, c);
    configurePoint(rect.vertices[2], w, -h, theta, c);
    configurePoint(rect.vertices[3], -w, -h, theta, c);
}

//
// \class Robot
// 
class Robot
{
    friend ostream& operator<<(ostream& stream, const Robot& mp);
    friend istream& operator>>(istream& stream, Robot& mp);
public:    
    Robot(void);
    
    //
    // construct a robot with specified geometry.
    // @param id unique id
    // @param x center x coord
    // @param y center y coord
    // @param width robot width
    // @param length robot length
    // @param theta robot orientation
    //
    Robot(int id, double x, double y, double width, double length, double theta)
    {
        m_id = id;
        m_center.m_x = x;
        m_center.m_y = y;
        m_length = length;
        m_width = width;
        m_theta = theta;
        commit(); 
    }
    
    ~Robot(void)
    {
    }
    
    //
    // @return unique id
    //
    int GetId(void) const
    {
        return m_id;
    }
    
    //
    // @return robot center
    //
    Point GetGoalCenter(void) const
    {
        return m_goal;
    }
    
    //
    // Set center for robot's goal
    //
    void SetGoalCenter(double x, double y)
    {
        m_goal.m_x = x;
        m_goal.m_y = y;
    }
    
    //
    // @return radius of robot's goal
    //
    double GetGoalRadius(void) const
    {
        return m_goalRadius;
    }
    
    // 
    // Set radius of robot's goal
    //
    void SetGoalRadius(double r)
    {
        m_goalRadius = r;
    }
    
    //
    // @return robot's center
    //
    Point GetRobotCenter(void) const
    {
        return m_center;	
    }
    
    //
    // Set Robot's center
    //
    void SetRobotCenter(const Point center)
    {
        m_center = center;
    }
    
    //
    // Get robot's length
    // 
    double GetRobotLength(void) const
    {
        return m_length;
    }
    
    //
    // Get robot's width
    //
    double GetRobotWidth(void) const
    {
        return m_width;
    }
    
    //
    // Get Robot's orientation.
    //
    double GetRobotOrientation(void) const
    {
        return m_theta;
    }
    
    //
    // Set Robot's orientation
    //
    void SetRobotOrientation(double theta)
    {
        m_theta = theta;
    }

    //
    // @return distance from robot to goal, based on robot center
    double GetDistanceFromRobotCenterToGoal(void) const
    {
        return point_distance(m_center, m_goal);
    }
    
    //
    // @return true if robot has reached goal.
    //
    bool HasRobotReachedGoal(void) const
    {
        return GetDistanceFromRobotCenterToGoal() <= GetGoalRadius();
    }
    
    // 
    // Set robot's state (ie. center and orientation)
    // @param c center
    // @param theta orientation
    //
    void SetRobotState(const Point c, const double theta)
    {
        m_center = c;
        m_theta = theta;
        commit();
    }
    
    //
    // Set robot's state using state object
    //
    void SetRobotState(const State sto)
    {
        m_center = sto.center;
        m_theta = sto.theta;
        commit();
    }

    //
    // @return bounding volume to use for collision detection.
    //
    const Rectangle2D& GetBoundingVolume() const
    {
        return m_rect;
    }
    
    // after modifying theta and/or center, commit to modify bounding volume
    void commit()
    {
        configureRectangle(m_rect, m_width, m_length, m_theta, m_center);
    }

    // for debugging: print description of robot state and shape
    void print()
    {
        cout << "(" << m_center.m_x << ", " << m_center.m_y << "), "
                  << "(" << m_width << ", " << m_length << "), "
                  << "(" << m_theta << "), "
                  << "(" << m_goal.m_x << ", " << m_goal.m_y << "), "
                  << "(" << m_goalRadius << ")";
    }
    
protected:
    // unique robot id (linked to planner id)
    int m_id;
    
    // robot center
    Point m_center;
    
    // robot length
    double m_length;
    
    // robot width
    double m_width;
    
    // robot orientation
    double m_theta;
    
    // robot goal center
    Point m_goal;
    
    // robot goal radius
    double m_goalRadius;
    
    // configured robot bounds
    Rectangle2D m_rect;

    friend class Graphics;
};

inline ostream& operator<<(ostream& stream, const Robot& rbot) {
    stream << rbot.m_center.m_x << " "
           << rbot.m_center.m_y << " "
           << rbot.m_width << " "
           << rbot.m_length << " "
           << rbot.m_theta << " "
           << rbot.m_goal.m_x << " "
           << rbot.m_goal.m_y << " "
            << rbot.m_goalRadius;
    return stream;
}

inline istream& operator>>(istream& stream, Robot& rbot) {
    stream >> rbot.m_center.m_x
           >> rbot.m_center.m_y
           >> rbot.m_length
           >> rbot.m_width
           >> rbot.m_theta
           >> rbot.m_goal.m_x
           >> rbot.m_goal.m_y
           >> rbot.m_goalRadius;
    return stream;
}


#endif

