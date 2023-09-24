#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "PxPhysicsAPI.h"
#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;
using namespace physx;

class GridCell
{
    public:
        uint32_t id;
        Vector3f position;
        Vector3f normal;
        Vector3f half_extents;
        Vector3f centre;
        GridCell(uint32_t id, const Vector3f& position, const Vector3f& normal, const Vector3f& cell_size, const Vector3f& cell_centre);
};

class OccupancyGrid
{
    private:
        Vector3f bb_origin;
        Vector3f bb_extents;
        Vector3f cell_size;
        int resolution;
        vector<GridCell> grid_cells;
        Vector3f cell_centre(uint32_t idx);
        vector<Vector3f> sample_points_in_triangle(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2);
    public:
        uint32_t idx_cell_at(const Vector3f& point);
        vector<int> reverse_cell_idx(uint32_t idx);
        vector<GridCell> get_grid_cells();
        bool is_cell_occupied(const Vector3f& point);
        OccupancyGrid(const MatrixX3f& vertices, const MatrixX3i& triangles, int resolution);
};