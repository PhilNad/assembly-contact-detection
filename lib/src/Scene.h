#pragma once

class Scene;

#include <vector>
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
        void shapesFromTetMesh(PxArray<PxVec3>* /*tetMeshVertices*/, PxArray<PxU32>* /*tetMeshIndices*/, PxArray<PxShape>* /*outShapes*/);
        void startupPhysics();
        void cleanupPhysics();
    public:
        Scene();
        ~Scene();
        void add_object(
            string /*id*/, 
            Matrix4f /*pose*/, 
            MatrixX3f /*vertices*/, 
            MatrixX3i /*triangles*/,
            bool /*is_fixed*/,
            float /*mass*/,
            Vector3f /*com*/,
            string /*material name*/
        );

        Matrix4f get_object_pose(string /*id*/);
        void step_simulation(float /*dt*/);
        void run_simulation(float /*dt*/, float /*end_time*/);
        vector<string> get_contacted_objects(string /*id*/);
        MatrixX3f get_contact_points(string /*id1*/, string /*id2*/);
        MatrixX3f get_tri_vertices(string /*id*/);
        MatrixX3i get_tri_triangles(string /*id*/);
        MatrixX3f get_tetra_vertices(string /*id*/);
        MatrixX4i get_tetra_indices(string /*id*/);
        MatrixX3f get_voxel_centres(string id);
        Vector3f get_voxel_side_lengths(string id);
};
