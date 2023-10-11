#pragma once

class Scene;

#include <vector>
#include <set>
#include <memory>
#include <iostream>
#include <string>
#include "PxPhysicsAPI.h"
#include <eigen3/Eigen/Eigen>
#include "Object.h"

using namespace std;
using namespace Eigen;

class Scene
{
    private:
        physx::PxSimulationEventCallback* gContactReportCallback = nullptr;
        vector<shared_ptr<Object>> object_ptrs;
        void startupPhysics();
        void cleanupPhysics();
    public:
        float max_distance_factor = 0.2;
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
        void set_max_distance_factor(float max_distance_factor);
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
