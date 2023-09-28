#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include "PxPhysicsAPI.h"
#include "extensions/PxTetMakerExt.h"
#include "geometry/PxSimpleTriangleMesh.h"
#include <eigen3/Eigen/Eigen>
#include "OccupancyGrid.h"

using namespace std;
using namespace Eigen;
using namespace physx;

class Object
{
    private:
        shared_ptr<OccupancyGrid> occupancy_grid;
    public:
        string id;
        Matrix4f pose;
        MatrixX3f tri_vertices;
        MatrixX3i tri_triangles;
        MatrixX3f tetra_vertices;
        MatrixX4i tetra_indices;
        bool is_fixed;
        float mass;
        Vector3f com;
        string material_name;
        float max_separation;
        Object(
            string id, 
            Matrix4f pose, 
            MatrixX3f vertices, 
            MatrixX3i triangles,
            bool is_fixed,
            float mass,
            Vector3f com,
            string material_name
        );
        ~Object();
        MatrixX3f get_voxel_centres();
        Vector3f get_voxel_side_lengths();
        void set_max_separation(float max_separation);
        void set_pose(Matrix4f pose);
        bool is_valid_pose_matrix(Matrix4f matrix);
        void set_tetra_mesh(PxArray<PxVec3> vertices, PxArray<PxU32> indices);
        void set_tri_mesh(PxSimpleTriangleMesh& simpleTriMesh);
        void set_tri_mesh(MatrixX3f& vertices, MatrixX3i& triangles);
        bool validate_mesh(PxSimpleTriangleMesh& surfaceMesh);
        PxSimpleTriangleMesh remesh_surface_trimesh(MatrixX3f in_vertices, MatrixX3i in_triangles);
        bool create_tetra_mesh(PxSimpleTriangleMesh& triSurfaceMesh, PxArray<PxVec3>& tetMeshVertices, PxArray<PxU32>& tetMeshIndices);
        bool create_tetra_convex_set(PxArray<PxVec3> tetVertices, PxArray<PxU32> tetIndices, PxArray<PxConvexMeshDesc>& convexMeshDescs);
        shared_ptr<OccupancyGrid> create_occupancy_grid(int resolution, int sampling_method);
        MatrixX3f get_world_vertices();
};