//
//  Robot.cpp
//  FinalProject
//

#include <iostream>
using std::cout;

#include "Robot.h"

// distance from starting point (x1,y1) to end point (x2, y2)
double 
point_distance(const double x1, const double y1, const double x2, const double y2)
{
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

// distance from starting point (x1,y1) to end point (x2, y2)
double 
point_distance(const Point& p1, const Point& p2)
{
    return point_distance(p1.m_x, p1.m_y, p2.m_x, p2.m_y);
}

// distance from starting point (x1,y1) to end point (x2, y2)
double 
point_distance(const Point& p1, const double x2, const double y2)
{
    return point_distance(p1.m_x, p1.m_y, x2, y2);
}


// ClosestPtPointSegment
// Based on code from book Realtime Collision Detection by Christer Ericson.
// Chapter 5.
void 
ClosestPtPointSegment(const Point& c, const Point& a, const Point& b, Point &d)
{
    Point ab = 
    {
        b.m_x - a.m_x,
        b.m_y - a.m_y
    };
    
    Point ac =
    {
        c.m_x - a.m_x,
        c.m_y - a.m_y
    };
    
    double t = dotprod(ac, ab) / dotprod(ab, ab);
    
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    
    d.m_x = a.m_x + t * ab.m_x;
    d.m_y = a.m_y + t * ab.m_y;
}

// Signed2DTriArea
// Based on code from book Realtime Collision Detection by Christer Ericson.
// Chapter 5.
double 
Signed2DTriArea(const Point& a, const Point& b, const Point& c)
{
    return (a.m_x - c.m_x) * (b.m_y - c.m_y) - (a.m_y - c.m_y) * (b.m_x - c.m_x);
}

// Based on code from book Realtime Collision Detection by Christer Ericson.
// Chapter 5. (see Test2DSegmentSegment)
// @return true if segments intersect
bool 
IsSegmentIntersection(const Point& a, const Point& b, 
                           const Point& c, const Point& d)
{
    double a1 = Signed2DTriArea(a, b, d);
    double a2 = Signed2DTriArea(a, b, c);
    
    if (a1 * a2 < 0.0)
    {
        float a3 = Signed2DTriArea(c, d, a);
        float a4 = a3 + a2 - a1;
        if (a3 * a4 < 0.0)
        {
            return true;
        }
    }
    return false;
}

bool 
IsCollisionRectangleCircle(const Rectangle2D& rect, const Point& c, double r)
{
    // check that distance between obs center and
    // each rect side is greater than obs radius
    // but robot could envelop the obstacle...
    // so do one last check to ensure obs center
    // is not bounded by the robot.
    Point d;
    
    ClosestPtPointSegment(c, rect.vertices[0], rect.vertices[1], d);
    if (point_distance(d, c.m_x, c.m_y) < r)
        return true;
    
    ClosestPtPointSegment(c, rect.vertices[1], rect.vertices[2], d);
    if (point_distance(d, c.m_x, c.m_y) < r)
        return true;
    
    ClosestPtPointSegment(c, rect.vertices[2], rect.vertices[3], d);
    if (point_distance(d, c.m_x, c.m_y) < r)
        return true;
    
    ClosestPtPointSegment(c, rect.vertices[3], rect.vertices[0], d);
    if (point_distance(d, c.m_x, c.m_y) < r)
        return true;
    
    if (rect.inBounds(&c))
        return true;
    
    return false;
}

// @return true if rectangle and rectangle overlap.
bool 
IsCollisionRectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2)
{
    for( int i = 0; i < 4; ++i)
        for ( int j = 0; j < 4; ++j)
        {
            if (IsSegmentIntersection(
                                      rect1.vertices[i], rect1.vertices[(i + 1) % 4], 
                                      rect2.vertices[j], rect2.vertices[(j + 1) % 4]))
                return true;
        }
    
    // since there are no segment intersections, we only need
    // to check one vertex on each rectangle to rule out containment.
    return rect1.inBounds(&rect2.vertices[0]) ||
    rect2.inBounds(&rect1.vertices[0]);
}

bool 
sameSide(const Point& v, const Point& p, const Point& t1, const Point& t2)
{
    // t2 - t1
    Point vec_side = 
    {
        t2.m_x - t1.m_x,
        t2.m_y - t1.m_y
    };
    
    // perpindicular to vec_side
    Point vec_perp = 
    {
        - vec_side.m_y,
        vec_side.m_x
    };
    
    // v - t1
    Point vec_v =
    {
        v.m_x - t1.m_x,
        v.m_y - t1.m_y
    };
    
    // p - t1
    Point vec_p =
    {
        p.m_x - t1.m_x,
        p.m_y - t1.m_y
    };
    
    double dp_p = dotprod(vec_perp, vec_p);
    double dp_v = dotprod(vec_perp, vec_v);
    
    return (dp_p >= 0 && dp_v >= 0) || (dp_p < 0 && dp_v < 0);
    
}

// dot product of 2 vectors
inline double 
dotprod(const Point a, const Point b)
{
    return (a.m_x * b.m_x + a.m_y * b.m_y);
}

// return true if point p is in rectangle bounds
bool 
Rectangle2D::
inBounds(const Point* p) const
{
    return sameSide(*p, vertices[0], vertices[1], vertices[2])
    && sameSide(*p, vertices[1], vertices[2], vertices[3])
    && sameSide(*p, vertices[2], vertices[3], vertices[0])
    && sameSide(*p, vertices[3], vertices[0], vertices[1]);
}

void
Rectangle2D::printCorners() const
{
    cout << "{ ";
    for (int i = 0; i < 4; ++i)
    {
        vertices[i].printCoords();
        cout << ", ";
    }
    cout << " }";
}

