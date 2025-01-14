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

//Forward declaration
class Scene;

class Object
{
    private:
        shared_ptr<PxVec3> tri_mesh_vertices;
        shared_ptr<PxU32> tri_mesh_indices;
        shared_ptr<PxTriangleMeshGeometry> tri_mesh_geometry;
    public:
        string id;
        Matrix4f pose;
        MatrixX3f tri_vertices;
        MatrixX3i tri_triangles;
        MatrixX3f tetra_vertices;
        MatrixX4i tetra_indices;
        MatrixX3f canary_sphere_positions;
        bool is_volumetric;
        bool is_fixed;
        float mass;
        Vector3f com;
        string material_name;
        float max_separation;
        Scene* scene;
        shared_ptr<PxSimpleTriangleMesh> tri_mesh;
        shared_ptr<OccupancyGrid> occupancy_grid;
        Object(
            Scene* scene,
            string id, 
            Matrix4f pose, 
            MatrixX3f vertices, 
            MatrixX3i triangles,
            bool is_volumetric,
            bool is_fixed,
            float mass,
            Vector3f com,
            string material_name
        );
        ~Object();
        bool has_tri_mesh();
        bool has_tetra_mesh();
        bool has_canary_spheres();
        MatrixX3f get_voxel_centres();
        Vector3f get_voxel_side_lengths();
        int get_grid_resolution();
        void set_max_separation(float max_separation);
        PxTransform get_pose();
        void reset_pose(Matrix4f new_pose);
        bool is_valid_pose_matrix(Matrix4f matrix);
        void set_tetra_mesh(MatrixX3f vertices, MatrixX4i indices);
        void set_tetra_mesh(PxArray<PxVec3> vertices, PxArray<PxU32> indices);
        void set_tri_mesh(PxSimpleTriangleMesh& simpleTriMesh);
        void set_tri_mesh(MatrixX3f& vertices, MatrixX3i& triangles);
        bool validate_mesh(PxSimpleTriangleMesh& surfaceMesh);
        void remesh_surface_trimesh();
        void create_tri_mesh_geometry();
        bool create_tetra_mesh(PxSimpleTriangleMesh& triSurfaceMesh, PxArray<PxVec3>& tetMeshVertices, PxArray<PxU32>& tetMeshIndices);
        bool create_tetra_convex_set(PxArray<PxVec3> tetVertices, PxArray<PxU32> tetIndices, PxArray<PxConvexMeshDesc>& convexMeshDescs);
        shared_ptr<OccupancyGrid> create_occupancy_grid(int resolution, int sampling_method);

        OrientedPoint get_closest_point_on_surface(Vector3f query_point);
};