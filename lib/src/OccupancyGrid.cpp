#include "OccupancyGrid.h"

/// @brief Build a grid cell that contains all necessary information for collision detection.
/// @param id The index of the cell in the grid.
/// @param position Position of the point that was sampled in the triangle, to be returned upon collision.
/// @param normal Normal of the triangle, to be returned upon collision.
/// @param cell_size Size of the cell along each dimension.
/// @param cell_centre Position of the centre of the cuboid that represents the cell.
GridCell::GridCell(uint32_t id, vector<float> position, vector<float> normal, vector<float> cell_size, vector<float> cell_centre)
{
    this->id = id;
    //NOTE: The position does not correspond to the centre of the cell
    // Intead, it is the position of the point that was sampled in the triangle
    this->position = position;
    this->normal = normal;

    //Record half-extents
    this->half_extents = {cell_size[0]/2, cell_size[1]/2, cell_size[2]/2};

    //Record centre
    this->centre = cell_centre;
}

/**
 * Determines whether a given point is inside a triangle defined by three vertices.
 *
 * @param point The point to check.
 * @param v0 The first vertex of the triangle.
 * @param v1 The second vertex of the triangle.
 * @param v2 The third vertex of the triangle.
 * @return True if the point is inside the triangle, false otherwise.
 */
bool point_inside_triangle(Vector3f point, Vector3f v0, Vector3f v1, Vector3f v2)
{
    // Compute barycentric weights
    Vector3f denom = v0.cross(v1) + v1.cross(v2) + v2.cross(v0);
    Vector3f weights = {v1.cross(v2) + point.cross(v1-v2), v2.cross(v0) + point.cross(v2-v0), v0.cross(v1) + point.cross(v0-v1)};
    weights /= denom;

    // Check if point is inside triangle
    return weights.minCoeff() >= 0 && weights.maxCoeff() <= 1 && weights.sum() <= 1;
}

/// @brief Sample points uniformly, and keep only those that are inside the triangle.
/// @param v0 First vertex of the triangle.
/// @param v1 Second vertex of the triangle.
/// @param v2 Third vertex of the triangle.
/// @param resolution Distance between sampled points.
/// @return Matrix with each row representing a sampled point.
MatrixX3f sample_points_in_triangle(Vector3f v0, Vector3f v1, Vector3f v2, float resolution)
{
    //Project the triangle onto the x-y plane
    Vector3f normal = (v1 - v0).cross(v2 - v0).normalized();
    Vector3f x_axis = (v1 - v0).normalized();
    Vector3f y_axis = normal.cross(x_axis).normalized();
    //Rotation matrix that projects points onto the x-y plane
    Matrix3f projection;
    projection << x_axis, y_axis, normal;
    //Each column is the coordinates of a vertex
    Matrix3f vertices_in_columns; = (MatrixX3f(3, 3) << v0, v1, v2).finished().transpose();
    //Each row is the coordinates of a vertex projected onto the x-y plane
    Matrix3f projected_vertices = (projection * vertices_in_columns).transpose();
    Vector3f v0_proj = projected_vertices.row(0);
    Vector3f v1_proj = projected_vertices.row(1);
    Vector3f v2_proj = projected_vertices.row(2);

    //Get the bounding box of the projected triangle
    float min_x = projected_vertices.col(0).minCoeff();
    float min_y = projected_vertices.col(1).minCoeff();
    float max_x = projected_vertices.col(0).maxCoeff();
    float max_y = projected_vertices.col(1).maxCoeff();

    //Sample points in the bounding box
    vector<Vector3f> points;
    for (float x = min_x; x <= max_x; x += resolution)
    {
        for (float y = min_y; y <= max_y; y += resolution)
        {
            //Check if point is inside triangle
            if (point_inside_triangle(Vector3f(x, y, 0), v0_proj, v1_proj, v2_proj))
            {
                Vector3f unprojected_point = projection.inverse() * Vector3f(x, y, 0);
                points.push_back(unprojected_point);
            }
        }
    }
}

/// @brief Check is the given point is in a cell that is already occupied.
/// @param point Query point, expressed in world coordinates.
/// @return True if the cell in which the point would fall is occupied, false otherwise.
bool OccupancyGrid::is_cell_occupied(Vector3f point){
    //Get cell indices
    int cell_idx = this->idx_cell_at(point);
    //Iterate over the grid cells
    for (int i = 0; i < this->grid_cells.size(); i++){
        //Check if the cell is occupied
        if (this->grid_cells[i].id == cell_idx){
            return true;
        }
    }
    return false;
}


/// @brief Returns the centre point of the cell at the given index.
/// @param idx The index of the cell to get the centre point of.
/// @return The centre point of the cell as a Vector3f.
Vector3f OccupancyGrid::cell_centre(uint32_t idx)
{
    //Get the coordinates of the cell
    Vector3f cell_coords = this->reverse_cell_idx(idx);
    //Get the position of the cell centre
    Vector3f cell_centre = this->bb_origin + cell_coords.cwiseProduct(this->cell_size) + this->cell_size/2;
    return cell_centre;
}

/// @brief Get the indices of the cell that contains a given point.
/// @param point 3x1 vector representing the point.
/// @return 3x1 vector representing the indices of the cell.
uint32_t OccupancyGrid::idx_cell_at(Vector3f point)
{
    // Get cell indices. The (i,j,k)th cell is the one that is reached
    // when moving i cells in the x direction, j cells in the y direction,
    // and k cells in the z direction from the origin of the bounding box.
    int i = (point[0] - this->bb_origin[0]) / this->cell_size[0];
    int j = (point[1] - this->bb_origin[1]) / this->cell_size[1];
    int k = (point[2] - this->bb_origin[2]) / this->cell_size[2];

    //There is this->resolution cells along each dimension
    uint32_t idx = i + j*this->resolution + k*this->resolution*this->resolution;

    // Return cell indices
    return idx;
}

/// @brief Get the coordinates of the cell with the given index.
/// @param idx Index of the cell.
/// @return 3x1 vector representing the coordinates of the cell.
Vector3f OccupancyGrid::reverse_cell_idx(uint32_t idx)
{
    //Perform the reverse operation of idx_cell_at which is
    //  int idx = i + j*this->resolution + k*this->resolution*this->resolution;

    // Get cell indices
    int i = idx % this->resolution;
    int j = (idx / this->resolution) % this->resolution;
    int k = idx / (this->resolution*this->resolution);

    // Return cell coordinates
    return {i, j, k};
}

/// @brief  Get the list of grid cells.
/// @return List of grid cells.
OccupancyGrid::get_grid_cells()
{
    return this->grid_cells;
}

/// @brief Builds a grid of cells that represent the volume of the object.
/// @param vertices Array of vertices of the triangle mesh representing the object.
/// @param triangles Array of indices of the triangle mesh representing the object.
/// @param resolution Desired number of cells per unit of length.
OccupancyGrid::OccupancyGrid(MatrixX3f vertices, MatrixX3i triangles, float resolution)
{
    // Get bounding box
    float min_x = vertices.col(0).minCoeff();
    float min_y = vertices.col(1).minCoeff();
    float min_z = vertices.col(2).minCoeff();
    float max_x = vertices.col(0).maxCoeff();
    float max_y = vertices.col(1).maxCoeff();
    float max_z = vertices.col(2).maxCoeff();

    // Get bounding box origin and extents
    this->bb_origin = {min_x, min_y, min_z};
    this->bb_extents = {max_x - min_x, max_y - min_y, max_z - min_z};

    // Get grid resolution
    this->resolution = {resolution, resolution, resolution};

    // Get grid cell size
    this->cell_size = {this->bb_extents[0] / this->resolution[0], this->bb_extents[1] / this->resolution[1], this->bb_extents[2] / this->resolution[2]};

    // Iterate over triangles
    for (int i = 0; i < triangles.rows(); i++)
    {
        // Get triangle vertices
        Vector3f v0 = vertices.row(triangles(i, 0));
        Vector3f v1 = vertices.row(triangles(i, 1));
        Vector3f v2 = vertices.row(triangles(i, 2));

        // Get triangle normal assuming that the indices are
        // ordered with the PhysX convention.
        Vector3f normal = (v1 - v0).cross(v2 - v0).normalized();

        // Sample points in the triangle
        vector<Vector3f> sampled_points = sample_points_in_triangle(v0, v1, v2, resolution);

        //For each sample point, check if a grid cell has already been created for it
        // If not, create a new grid cell
        for (int j = 0; j < sampled_points.size(); j++){
            bool occupied = this->is_cell_occupied(sampled_points[j]);
            if(!occupied){
                //Get the cell indices
                int cell_idx = this->idx_cell_at(sampled_points[j]);
                //Get the centre of the cell
                Vector3f cell_centre = this->cell_centre(cell_idx);
                //Create the grid cell
                GridCell grid_cell = GridCell(cell_idx, sampled_points[j], normal, this->cell_size, cell_centre);
                //Add the grid cell to the list of grid cells
                this->grid_cells.push_back(grid_cell);
            }
        }
    }
}
        