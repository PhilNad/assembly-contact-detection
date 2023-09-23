#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "PxPhysicsAPI.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;
using namespace physx;

class Object
{
    private:
        Matrix4f pose;
        MatrixX3f tri_vertices;
        MatrixX3i tri_triangles;
        MatrixX3f tetra_vertices;
        MatrixX4i tetra_indices;
        bool is_fixed;
        float mass;
        Vector3f com;
        string material_name;
    public:
        string id;
        float max_separation;
        Object(
            string /*id*/, 
            Matrix4f /*pose*/, 
            MatrixX3f /*vertices*/, 
            MatrixX3i /*triangles*/,
            bool /*is_fixed*/,
            float /*mass*/,
            Vector3f /*com*/,
            string /*material name*/
        );
        ~Object();
        string get_id();
        Matrix4f get_pose();
        MatrixX3f get_tri_vertices();
        MatrixX3i get_tri_triangles();
        MatrixX3f get_tetra_vertices();
        MatrixX4i get_tetra_indices();
        bool get_is_fixed();
        float get_mass();
        Vector3f get_com();
        string get_material_name();
        void set_max_separation(float /*max_separation*/);
        void set_pose(Matrix4f /*pose*/);
        void set_vertices(MatrixX3f /*vertices*/);
        void set_triangles(MatrixX3i /*triangles*/);
        void set_is_fixed(bool /*is_fixed*/);
        void set_mass(float /*mass*/);
        void set_com(Vector3f /*com*/);
        void set_material_name(string /*material_name*/);
        void set_tetra_mesh(PxArray<PxVec3> /*vertices*/, PxArray<PxU32> /*indices*/);
        void set_tri_mesh(PxSimpleTriangleMesh& /*simpleTriMesh*/);
        MatrixX3f get_world_vertices();
};