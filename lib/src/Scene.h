#pragma once

class Scene;

#include <vector>
#include <set>
#include <memory>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Eigen>
#include "AssemblyCD.h"

using namespace std;
using namespace Eigen;

struct LineSegmentIntersection {
    Vector2f intersection_point_1 = Vector2f(NAN, NAN);
    Vector2f intersection_point_2 = Vector2f(NAN, NAN);
    int nb_intersections = 0;
};
LineSegmentIntersection line_segment_intersection(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2, bool closed_segments = true);
vector<Vector2f> edge_triangle_intersection(const Vector2f& edge_p0, const Vector2f& edge_p1, Triangle<Vector2f>& triangle);
PointSet2D line_AARectangle_intersection(const Vector2f& segment_p0, const Vector2f& segment_p1, AARectangle& rectangle);
vector<Vector3f> triangle_overlap_over_AARectangle(AARectangle& aarec, shared_ptr<Triangle<Vector3f>> t1, shared_ptr<Triangle<Vector3f>> t2, bool boundary_included = false);

class Scene
{
    private:
        PxSimulationEventCallback* gContactReportCallback = nullptr;
        vector<shared_ptr<Object>> object_ptrs;
        void startupPhysics();
        void cleanupPhysics();
    public:
        Scene();
        ~Scene();
        void add_object(
            string id, 
            Matrix4f pose, 
            MatrixX3f vertices, 
            MatrixX3i triangles,
            int resolution = 15,
            bool is_fixed = false,
            float mass = 1,
            Vector3f com = Vector3f::Zero(),
            string material_name = "wood"
        );

        Matrix4f get_object_pose(string id);
        void step_simulation(float dt);
        set<string> get_contacted_objects(string target_object);
        MatrixX3f get_contact_points(string id1, string id2);
        void merge_similar_contact_points(float position_threshold, float normal_threshold);
        MatrixX3f get_tri_vertices(string id);
        MatrixX3i get_tri_triangles(string id);
        MatrixX3f get_tetra_vertices(string id);
        MatrixX4i get_tetra_indices(string id);
        MatrixX3f get_voxel_centres(string id);
        Vector3f get_voxel_side_lengths(string id);
};
