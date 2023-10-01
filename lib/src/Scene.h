#pragma once

class Scene;

#include <vector>
#include <set>
#include <memory>
#include <iostream>
#include <string>
#include <eigen3/Eigen/Eigen>
#include "Object.h"
#include "Contact.h"
#include "OccupancyGrid.h"

using namespace std;
using namespace Eigen;

class Scene
{
    private:
        PxSimulationEventCallback* gContactReportCallback = nullptr;
        vector<shared_ptr<Object>> object_ptrs;
        PxSimpleTriangleMesh remesh_surface_trimesh(MatrixX3f in_vertices, MatrixX3i in_triangles);
        bool create_tetra_mesh(PxSimpleTriangleMesh& triSurfaceMesh, PxArray<PxVec3>& tetMeshVertices, PxArray<PxU32>& tetMeshIndices);
        bool create_tetra_convex_set(PxArray<PxVec3> tetVertices, PxArray<PxU32> tetIndices, PxArray<PxConvexMeshDesc>& convexMeshDescs);
        bool validate_mesh(PxSimpleTriangleMesh& surfaceMesh);
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
        void run_simulation(float dt, float end_time);
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
