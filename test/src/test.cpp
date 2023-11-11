#include <iostream>
#include <vector>
#include <set>
#include "PxPhysicsAPI.h"
#include "eigen3/Eigen/Eigen"
#include "AssemblyCD.h"

using namespace physx;
using namespace std;

class Cube{
    public:
        MatrixX3f vertices;
        MatrixX3i triangles;
        Cube(float extent_x, float extent_y, float extent_z){
            float hex = abs(extent_x)/2;
            float hey = abs(extent_y)/2;
            float hez = abs(extent_z)/2;
            vertices = MatrixXf::Zero(8, 3);
            vertices.row(0) << -hex, -hey, -hez;
            vertices.row(1) << -hex,  hey, -hez;
            vertices.row(2) <<  hex,  hey, -hez;
            vertices.row(3) <<  hex, -hey, -hez;
            vertices.row(4) << -hex, -hey,  hez;
            vertices.row(5) << -hex,  hey,  hez;
            vertices.row(6) <<  hex,  hey,  hez;
            vertices.row(7) <<  hex, -hey,  hez;

            triangles = MatrixXi::Zero(12, 3);
            //Top
            triangles.row(0) << 7, 6, 4;
            triangles.row(1) << 4, 6, 5;
            //Bottom
            triangles.row(2) << 0, 2, 3;
            triangles.row(3) << 2, 0, 1;
            //Side X+
            triangles.row(4) << 3, 2, 7;
            triangles.row(5) << 7, 2, 6;
            //Side X-
            triangles.row(6) << 4, 1, 0;
            triangles.row(7) << 1, 4, 5;
            //Side Y+
            triangles.row(8) << 5, 2, 1;
            triangles.row(9) << 5, 6, 2;
            //Side Y-
            triangles.row(10) << 3, 4, 0;
            triangles.row(11) << 4, 3, 7;
        }
        ~Cube(){}
        void translate(float x, float y, float z){
            for(int i = 0; i < vertices.rows(); i++){
                vertices.row(i) = vertices.row(i) + RowVector3f(x, y, z);
            }
        }
        void scale(float scaling, RowVector3f centre){
            for(int i = 0; i < vertices.rows(); i++){
                vertices.row(i) = abs(scaling)*(vertices.row(i) - centre) + centre;
            }
        }
        void rotate(float angle, RowVector3f axis, RowVector3f centre){
            Matrix3f rotation_matrix;
            rotation_matrix = AngleAxisf(angle, axis.normalized());
            for(int i = 0; i < vertices.rows(); i++){
                vertices.row(i) = (rotation_matrix*(vertices.row(i) - centre).transpose()).transpose() + centre;
            }
        }
};


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

/// @brief Expressed the given vertices in the object frame.
/// @param pose Homogeneous pose matrix of the object.
/// @param vertices_world Nx3 matrix of vertices in the world frame.
/// @return Nx3 matrix of vertices in the object frame.
MatrixX3f vertices_to_object_frame(Matrix4f pose, MatrixX3f vertices_world)
{
    Matrix4f pose_inv = pose.inverse();
    Matrix3f pose_inv_R = pose_inv.block<3, 3>(0, 0); //Rotation
    Vector3f pose_inv_t = pose_inv.block<3, 1>(0, 3); //Translation
    MatrixX3f vertices_o = (pose_inv_R*vertices_world.transpose()).transpose().rowwise() + pose_inv_t.transpose();
    return vertices_o;
}

int main(int argc, char** argv) {
    //test_triangle_triangle_intersections();
    //return 0;
    // Initialize the scene
    Scene scene;

    // Add a cube to the scene
    string id = "cube1";
    Matrix4f pose;
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube cube1 = Cube(1, 1, 1);
    scene.add_object(id, pose, cube1.vertices, cube1.triangles);
    cube1.translate(pose(0, 3), pose(1, 3), pose(2, 3));

    // Add another cube to the scene
    id = "cube2";
    pose << 1, 0, 0, 0,
            0, 1, 0, 0.25,
            0, 0, 1, 1.5,
            0, 0, 0, 1;
    Cube cube2 = Cube(1, 1, 1);
    scene.add_object(id, pose, cube2.vertices, cube2.triangles, 30);
    cube2.rotate(0.79, RowVector3f(0, 0, 1), RowVector3f(0, 0, 0));
    cube2.translate(pose(0, 3), pose(1, 3), pose(2, 3));
    

    // Add a cube to the scene
    id = "cube3";
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube cube3 = Cube(1, 1, 1);
    scene.add_object(id, pose, cube3.vertices, cube3.triangles, 30);
    cube3.translate(pose(0, 3), pose(1, 3), pose(2, 3));

    //Remove cube1
    scene.remove_object("cube1");

    //Move cube2
    pose << 1, 0, 0, 0,
            0, 1, 0, 2.25,
            0, 0, 1, 1.4,//1.5
            0, 0, 0, 1;
    scene.set_object_pose("cube2", pose);

    //Move cube3
    pose << 1, 0, 0, 0,
            0, 1, 0, 2,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    scene.set_object_pose("cube3", pose);

    // Get the list of objects in contact with the cube
    set<string> contacted_objects = scene.get_contacted_objects("cube2");
    cout << "Contacted objects: ";
    for(auto it = contacted_objects.begin(); it != contacted_objects.end(); ++it){
        cout << *it << " ";
    }
    cout << endl;

    // Get the contact points between the two objects
    MatrixX3f contact_points = scene.get_all_contact_points("cube2");
    cout << "Found " << contact_points.rows() << " contact points." << endl;
    contact_points = scene.get_all_penetrating_contact_points("cube2");
    cout << "Found " << contact_points.rows() << " penetrating points." << endl;
    for (int i = 0; i < contact_points.rows(); i++) {
        //cout << contact_points.row(i) << endl;
    }

    MatrixX3f hull_contacts = scene.get_contact_convex_hull("cube2");
    cout << "Found " << hull_contacts.rows() << " hull contact points." << endl;

    return 0;
}