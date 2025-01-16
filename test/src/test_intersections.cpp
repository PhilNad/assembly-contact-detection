#include <iostream>
#include <vector>
#include <set>
#include "PxPhysicsAPI.h"
#include "eigen3/Eigen/Eigen"
#include "AssemblyCD.h"

void test_line_segments_intersection()
{
    LineSegmentIntersection result;
    Vector2f p1, q1, p2, q2;

    cout << "Non-collinear and intersecting" << endl;
    p1 << 0.5, 0; q1 << 0.5, 1;
    p2 << 0, 0.5; q2 << 1, 0.5;
    result = line_segment_intersection(p1, q1, p2, q2);
    cout << "Line 1: (" << p1.transpose() << "), (" << q1.transpose() << ") Line 2: (" << p2.transpose() << "), (" << q2.transpose() << ")" << endl;
    cout << "Intersection: " << result.nb_intersections << endl;
    cout << "Intersection point 1: " << result.intersection_point_1.transpose() << endl;
    cout << "Intersection point 2: " << result.intersection_point_2.transpose() << endl;
    cout << "---" << endl;

    cout << "Collinear and overlapping" << endl;
    p1 << 0, 0.5; q1 << 0, 1;
    p2 << 0, 0;   q2 << 0, 1;
    result = line_segment_intersection(p1, q1, p2, q2);
    cout << "Line 1: (" << p1.transpose() << "), (" << q1.transpose() << ") Line 2: (" << p2.transpose() << "), (" << q2.transpose() << ")" << endl;
    cout << "Intersection: " << result.nb_intersections << endl;
    cout << "Intersection point 1: " << result.intersection_point_1.transpose() << endl;
    cout << "Intersection point 2: " << result.intersection_point_2.transpose() << endl;
    cout << "---" << endl;

    cout << "Collinear and disjoint" << endl;
    p1 << 0, 1; q1 << 0, 0.75;
    p2 << 0, 0; q2 << 0, 0.5;
    result = line_segment_intersection(p1, q1, p2, q2);
    cout << "Line 1: (" << p1.transpose() << "), (" << q1.transpose() << ") Line 2: (" << p2.transpose() << "), (" << q2.transpose() << ")" << endl;
    cout << "Intersection: " << result.nb_intersections << endl;
    cout << "Intersection point 1: " << result.intersection_point_1.transpose() << endl;
    cout << "Intersection point 2: " << result.intersection_point_2.transpose() << endl;
    cout << "---" << endl;

    cout << "Non-collinear and Non-intersecting" << endl;
    p1 << 0, 0; q1 << 0, 0.5;
    p2 << 1, 0; q2 << 1, 1;
    result = line_segment_intersection(p1, q1, p2, q2);
    cout << "Line 1: (" << p1.transpose() << "), (" << q1.transpose() << ") Line 2: (" << p2.transpose() << "), (" << q2.transpose() << ")" << endl;
    cout << "Intersection: " << result.nb_intersections << endl;
    cout << "Intersection point 1: " << result.intersection_point_1.transpose() << endl;
    cout << "Intersection point 2: " << result.intersection_point_2.transpose() << endl;
    cout << "---" << endl;
}

void test_segment_triangle_intersection()
{
    Triangle<Vector2f> triangle(Vector2f(0, 0), Vector2f(1, 0), Vector2f(0, 1));
    /*
        (0,1)
        | \
        |   \
        |    \
        |_____\
     (0,0)   (1,0)
    */
    Vector2f p1, q1;
    PointSet2D intersections;

    cout << "Non-collinear and intersecting at one end of the segment" << endl;
    p1 << 0.5, 0.5; q1 << 0.5, 1.5;
    cout << "Line: (" << p1.transpose() << "), (" << q1.transpose() << ")" << endl;
    intersections = edge_triangle_intersection(p1, q1, triangle);
    for(auto& point : intersections){
        cout << point.transpose() << endl;
    }
    cout << "---" << endl;

    cout << "Non-collinear and intersecting two sides (going in and out)" << endl;
    p1 << -0.25, 0.5; q1 << 0.75, 0.5;
    cout << "Line: (" << p1.transpose() << "), (" << q1.transpose() << ")" << endl;
    intersections = edge_triangle_intersection(p1, q1, triangle);
    for(auto& point : intersections){
        cout << point.transpose() << endl;
    }
    cout << "---" << endl;

    cout << "Non-collinear and intersecting one side (going in)" << endl;
    p1 << -0.25, 0.5; q1 << 0.25, 0.5;
    cout << "Line: (" << p1.transpose() << "), (" << q1.transpose() << ")" << endl;
    intersections = edge_triangle_intersection(p1, q1, triangle);
    for(auto& point : intersections){
        cout << point.transpose() << endl;
    }
    cout << "---" << endl;

    cout << "Collinear and overlapping" << endl;
    p1 << 0, 0; q1 << 0, 1;
    cout << "Line: (" << p1.transpose() << "), (" << q1.transpose() << ")" << endl;
    intersections = edge_triangle_intersection(p1, q1, triangle);
    for(auto& point : intersections){
        cout << point.transpose() << endl;
    }
    cout << "---" << endl;
    
    cout << "Inside triangle" << endl;
    p1 << 0.25, 0.5; q1 << 0.5, 0.25;
    cout << "Line: (" << p1.transpose() << "), (" << q1.transpose() << ")" << endl;
    intersections = edge_triangle_intersection(p1, q1, triangle);
    for(auto& point : intersections){
        cout << point.transpose() << endl;
    }
    cout << "---" << endl;

    cout << "Outside triangle" << endl;
    p1 << 0.25, 1; q1 << 0.75, 0.5;
    cout << "Line: (" << p1.transpose() << "), (" << q1.transpose() << ")" << endl;
    intersections = edge_triangle_intersection(p1, q1, triangle);
    for(auto& point : intersections){
        cout << point.transpose() << endl;
    }
    cout << "---" << endl;
}

void test_triangle_triangle_intersections()
{
    /*
    AARectangle: Plane: (0, 0, 1, 1) Centre: (0.508839, 0.508333, 1) Half Extents: (0.00883888, 0.00833334, 0)
    Rectangle: (0.5, 0.5, 1)--(0.517678, 0.5, 1)  (0.517678, 0.5, 1)--(0.517678, 0.516667, 1)  (0.517678, 0.516667, 1)--(0.5, 0.516667, 1)  (0.5, 0.516667, 1)--(0.5, 0.5, 1)  
    Triangle 1:(0, 0, 1)--(1, 0, 1)  (1, 0, 1)--(1, 1, 1)  (1, 1, 1)--(0, 0, 1)  
    Triangle 2:(-0.207107, 0, 1)--(0.5, 0, 1)  (0.5, 0, 1)--(0.5, 1, 1)  (0.5, 1, 1)--(-0.207107, 0, 1)    
    Rectangle area: 0.000294629
    Triangle 1 area: 0.5
    Triangle 2 area: 0.353553
    No intersection between the rectangle-triangle1 intersection and the second triangle.
    */

    AARectangle r1(PxPlane(0, 0, 1, 1), Vector3f(0.508839, 0.508333, 1), Vector3f(0.00883888, 0.00833334, 0));

    Triangle<Vector3f> t1(Vector3f(0, 0, 1), Vector3f(1, 0, 1), Vector3f(1, 1, 1));
    Triangle<Vector3f> t2(Vector3f(-0.207107, 0, 1), Vector3f(0.5, 0, 1), Vector3f(0.5, 1, 1));

    shared_ptr<Triangle<Vector3f>> t1_ptr = make_shared<Triangle<Vector3f>>(t1);
    shared_ptr<Triangle<Vector3f>> t2_ptr = make_shared<Triangle<Vector3f>>(t2);

    PointSet3D intersections = triangle_triangle_AARectangle_intersection(r1, t1_ptr, t2_ptr, r1.half_extents[0]);
}

void test_segment_rectangle_intersection()
{
    AARectangle aarec(PxPlane(0, 0, 1, 1), Vector3f(0, 0, 0), Vector3f(1, 1, 1));
    Vector2f start, end;
    PointSet2D line_rect_intersections;

    cout << "Segment overlapping one side." << endl;
    start << -1.5, -1; end << 0, -1;
    line_rect_intersections = line_AARectangle_intersection(start, end, aarec);
    for(auto it = line_rect_intersections.begin(); it != line_rect_intersections.end(); ++it){
        cout << (*it).transpose() << endl;
    }
    cout << "Should be (-1,-1), (0, -1)" << endl;
    cout << "---" << endl;

    cout << "Segment with one end on the boundary." << endl;
    start << 1, 1; end << 2, 2;
    line_rect_intersections = line_AARectangle_intersection(start, end, aarec);
    for(auto it = line_rect_intersections.begin(); it != line_rect_intersections.end(); ++it){
        cout << (*it).transpose() << endl;
    }
    cout << "Should be (1, 1)" << endl;
    cout << "---" << endl;

    cout << "Segment inside rectangle." << endl;
    start << 0.5, 0.5; end << -0.5, -0.5;
    line_rect_intersections = line_AARectangle_intersection(start, end, aarec);
    for(auto it = line_rect_intersections.begin(); it != line_rect_intersections.end(); ++it){
        cout << (*it).transpose() << endl;
    }
    cout << "Should be (0.5, 0.5), (-0.5, -0.5)" << endl;
    cout << "---" << endl;

    cout << "Segment outside rectangle." << endl;
    start << 2, 0.5; end << 2, 1.5;
    line_rect_intersections = line_AARectangle_intersection(start, end, aarec);
    for(auto it = line_rect_intersections.begin(); it != line_rect_intersections.end(); ++it){
        cout << (*it).transpose() << endl;
    }
    cout << "Should be empty." << endl;
    cout << "---" << endl;

    cout << "Segment going through rectangle." << endl;
    start << -1.5, 0; end << 0, 1.5;
    line_rect_intersections = line_AARectangle_intersection(start, end, aarec);
    for(auto it = line_rect_intersections.begin(); it != line_rect_intersections.end(); ++it){
        cout << (*it).transpose() << endl;
    }
    cout << "Should be (-1, 0.5), (-0.5, 1)" << endl;
    cout << "---" << endl;

    cout << "Segment intersecting one side." << endl;
    start << 0.5, 0.5; end << 0.5, 1.5;
    line_rect_intersections = line_AARectangle_intersection(start, end, aarec);
    for(auto it = line_rect_intersections.begin(); it != line_rect_intersections.end(); ++it){
        cout << (*it).transpose() << endl;
    }
    cout << "Should be (0.5, 0.5), (0.5, 1)" << endl;
    cout << "---" << endl;
}