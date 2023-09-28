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
        Vector3f half_extents;
        Vector3f centre;
        GridCell(uint32_t id, shared_ptr<OrientedPoint> surface_point, const Vector3f& cell_size, const Vector3f& cell_centre);
        void additional_point(shared_ptr<OrientedPoint> surface_point);
        OrientedPoint weighted_average(const Vector3f& query_point);
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