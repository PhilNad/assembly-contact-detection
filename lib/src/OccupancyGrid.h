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
        vector<float> position;
        vector<float> normal;
        vector<float> half_extents;
        vector<float> centre;
        GridCell(uint32_t id, vector<float> position, vector<float> normal, vector<float> cell_size, vector<float> cell_centre);
};

class OccupancyGrid
{
    private:
        vector<float> bb_origin;
        vector<float> bb_extents;
        vector<float> cell_size;
        vector<float> resolution;
        vector<GridCell> grid_cells;
        Vector3f OccupancyGrid::cell_centre(uint32_t idx);
    public:
        uint32_t idx_cell_at(Vector3f point);
        Vector3f reverse_cell_idx(uint32_t idx);
        vector<GridCell> get_grid_cells();
        bool is_cell_occupied(Vector3f point);
        OccupancyGrid(MatrixX3f vertices, MatrixX3i triangles, float resolution);
};