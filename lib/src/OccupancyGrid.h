#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <unordered_map>
#include "PxPhysicsAPI.h"
#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;
using namespace physx;

template <typename T>
class Triangle
{
    private:
        struct point_inside_triangle_computations_cache {
            bool active = false;
            T v0r;
            T v1r;
            float d00;
            float d01;
            float d11;
            float inv_denom;
        };
        point_inside_triangle_computations_cache pt_in_tri_cache;
        void compute_signed_area();
    public:
        T vertex_0;
        T vertex_1;
        T vertex_2;
        float signed_area = 0;
        Triangle(T vertex_0, T vertex_1, T vertex_2);
        bool contains(const T& point, bool boundary_included);
};

class AARectangle
{
    private:
        Vector3f u;
        Vector3f v;
        float EPSILON = 1e-3;
    public:
        PxPlane plane;
        Vector3f centre;
        Vector3f half_extents;
        float area;
        Vector2f u_extents;
        Vector2f v_extents;
        AARectangle(PxPlane plane, Vector3f centre, Vector3f half_extents);
        Vector2f project_point(const Vector3f& point);
        Vector3f unproject_point(const Vector2f& point);
        Triangle<Vector2f> project_triangle(const Triangle<Vector3f>& triangle);
        bool contains(const Vector3f& point);
        bool contains(const Vector2f& point);
        Vector2f project_and_clamp(const Vector3f& point);
        Vector2f inside_or_on(const Vector2f& point);
        float get_min_u();
        float get_max_u();
        float get_min_v();
        float get_max_v();
};

struct OrientedPoint
{
    Vector3f position;
    Vector3f normal;
};

class GridCell
{
    public:
        uint32_t id;
        vector<shared_ptr<OrientedPoint>> surface_points;
        vector<shared_ptr<Triangle<Vector3f>>> triangles;
        Vector3f half_extents;
        Vector3f centre;
        GridCell(uint32_t id, shared_ptr<OrientedPoint> surface_point, const Vector3f& cell_size, const Vector3f& cell_centre, shared_ptr<Triangle<Vector3f>> triangle);
        void additional_point(shared_ptr<OrientedPoint> surface_point);
        void additional_triangle(shared_ptr<Triangle<Vector3f>> triangle);
        OrientedPoint weighted_average(const Vector3f& query_point);
        AARectangle gridcell_to_gridcell_intersection(const GridCell& other);
};


class OccupancyGrid
{
    private:
        Vector3f bb_origin;
        Vector3f bb_extents;
        Vector3f cell_size;
        int resolution;
        unordered_map<uint32_t, GridCell> grid_cells;
        Vector3f cell_centre(uint32_t idx);
        
        struct point_inside_triangle_computations_cache {
            bool active = false;
            Vector3f v0 = Vector3f::Zero();
            Vector3f v1 = Vector3f::Zero();
            Vector3f v2 = Vector3f::Zero();
            Vector3f v0r;
            Vector3f v1r;
            float d00;
            float d01;
            float d11;
            float inv_denom;
        };
        bool point_inside_triangle(const Vector3f& p, const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, point_inside_triangle_computations_cache& cache);
        vector<Vector3f> sample_uniform_points_in_triangle(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, float density);
        vector<Vector3f> sample_random_points_in_triangle(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2, float density);
    public:
        uint32_t idx_cell_at(const Vector3f& point);
        vector<int> reverse_cell_idx(uint32_t idx);
        unordered_map<uint32_t, GridCell>* get_grid_cells();
        uint32_t is_cell_occupied(const Vector3f& point);
        uint32_t is_cell_occupied(uint32_t cell_idx);
        enum sampling_method {uniform, random};
        OccupancyGrid(const MatrixX3f& vertices, const MatrixX3i& triangles, int resolution, int sampling_method);
};