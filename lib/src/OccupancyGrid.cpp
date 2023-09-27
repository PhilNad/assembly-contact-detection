#include "OccupancyGrid.h"

/// @brief Build a grid cell that contains all necessary information for collision detection.
/// @param id The index of the cell in the grid.
/// @param surface_point Oriented point (position and normal) that was sampled on the surface.
/// @param cell_size Size of the cell along each dimension.
/// @param cell_centre Position of the centre of the cuboid that represents the cell.
GridCell::GridCell(uint32_t id, shared_ptr<OrientedPoint> surface_point, const Vector3f& cell_size, const Vector3f& cell_centre)
{
    this->id = id;
    //NOTE: The position does not correspond to the centre of the cell
    // Intead, it is the position of the point that was sampled in the triangle
    this->surface_points.push_back(surface_point);

    //Record half-extents
    this->half_extents = {cell_size[0]/2, cell_size[1]/2, cell_size[2]/2};

    //Record the world frame coordinates of the centre of the cell
    this->centre = cell_centre;
}

/// @brief Add information about a surface point to the grid cell.
/// @param surface_point Oriented point (position and normal) that was sampled on the surface.
void GridCell::additional_point(shared_ptr<OrientedPoint> surface_point){
    this->surface_points.push_back(surface_point);
}

/// @brief Compute the weighted average of the surface points positions and normals based on their distance to the query point.
/// @param query_point World frame coordinates of the query point.
/// @return Oriented point (position and normal) that is the weighted average of the surface points.
/// @note This should work well only if the grid cells are small enough. In that case, intersections with edges is possible.
OrientedPoint GridCell::weighted_average(const Vector3f& query_point){
    //Compute the distance between the query point and each surface point
    vector<float> distances;
    for (int i = 0; i < this->surface_points.size(); i++){
        distances.push_back((query_point - this->surface_points[i]->position).norm());
    }
    //Compute the weights of each surface point
    vector<float> weights;
    float sum = 0;
    for (int i = 0; i < distances.size(); i++){
        float weight = 1/distances[i];
        weights.push_back(weight);
        sum += weight;
    }
    //Compute the weighted average of the surface points
    Vector3f position_average = Vector3f::Zero();
    for (int i = 0; i < this->surface_points.size(); i++){
        position_average += weights[i]/sum * this->surface_points[i]->position;
    }
    //Compute the weighted average of the surface normals
    Vector3f normal_average = Vector3f::Zero();
    for (int i = 0; i < this->surface_points.size(); i++){
        normal_average += weights[i]/sum * this->surface_points[i]->normal;
    }

    //Create a new OrientedPoint
    OrientedPoint weighted_average;
    weighted_average.position = position_average;
    weighted_average.normal   = normal_average;

    return weighted_average;
}

/// @brief Determines whether a given point is inside a triangle defined by three vertices.
/// @param point The point to check.
/// @param v0 The first vertex of the triangle.
/// @param v1 The second vertex of the triangle.
/// @param v2 The third vertex of the triangle.
/// @return True if the point is inside the triangle, false otherwise.
/// @note See: https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Barycentric_coordinates_on_triangles
/// @note See: Real-Time Collision Detection page 137.
bool point_inside_triangle(const Vector3f& p, const Vector3f& v0, const Vector3f& v1, const Vector3f& v2)
{
    //Tolerance such that points directly on the triangle edge are considered to be inside the triangle
    float tol = 1e-6;

    //Express points relative to first vertex
    Vector3f v0r = v1 - v0;
    Vector3f v1r = v2 - v0;
    Vector3f v2r = p - v0;
    //Compute dot products
    float d00 = v0r.dot(v0r);
    float d01 = v0r.dot(v1r);
    float d11 = v1r.dot(v1r);
    float d20 = v2r.dot(v0r);
    float d21 = v2r.dot(v1r);
    //This is two times the triangle area
    float inv_denom = 1/(d00 * d11 - d01 * d01);
    //Barycentric weights that each describe the area of a sub-triangle over the area of the triangle
    // so they must be between 0 and 1. The weights also describe how much each vertex attracts the point.
    float alpha = (d11 * d20 - d01 * d21) * inv_denom;
    float beta  = (d00 * d21 - d01 * d20) * inv_denom;
    float gamma = 1.0f - alpha - beta;
    //Check if point is inside triangle
    return ((0 <= alpha+tol) && (alpha-tol <= 1) &&
            (0 <= beta+tol)  && (beta-tol  <= 1) &&
            (0 <= gamma+tol) && (gamma-tol <= 1));
}

/// @brief Sample points uniformly, and keep only those that are inside the triangle.
/// @param v0 First vertex of the triangle.
/// @param v1 Second vertex of the triangle.
/// @param v2 Third vertex of the triangle.
/// @param resolution Distance between sampled points.
/// @return Matrix with each row representing a sampled point.
vector<Vector3f> OccupancyGrid::sample_points_in_triangle(const Vector3f& v0, const Vector3f& v1, const Vector3f& v2)
{
    //Make sure that each edge of the triangle is non-zero
    assert((v1 - v0).norm() > 0);
    assert((v2 - v0).norm() > 0);
    assert((v2 - v1).norm() > 0);
    //OccupancyGrid should be initialized
    assert(this->cell_size[0] > 1e-12 && this->cell_size[1] > 1e-12 && this->cell_size[2] > 1e-12);

    //Express vertices relative to v0
    Vector3f v0r;
    v0r << 0, 0, 0;
    Vector3f v1r = v1 - v0;
    Vector3f v2r = v2 - v0;

    //Project the triangle onto the x-y plane
    Vector3f normal = v1r.cross(v2r).normalized();
    Vector3f x_axis = v1r.normalized();
    Vector3f y_axis = normal.cross(x_axis).normalized();
    //Rotation matrix that projects points onto the x-y plane
    Matrix3f projection = (MatrixX3f(3, 3) << x_axis, y_axis, normal).finished();
    //Matrix3f is initialized column by column and needs to be transposed to get the desired rotation matrix
    projection.transposeInPlace();
    //Each column is the coordinates of a vertex
    Matrix3f vertices_in_columns = (MatrixX3f(3, 3) << v0r, v1r, v2r).finished();
    //Each row is the coordinates of a vertex projected onto the x-y plane
    Matrix3f projected_vertices = projection * vertices_in_columns;
    Vector3f v0_proj = projected_vertices.col(0);
    Vector3f v1_proj = projected_vertices.col(1);
    Vector3f v2_proj = projected_vertices.col(2);

    //Get the bounding box of the projected triangle
    float min_x = projected_vertices.row(0).minCoeff();
    float min_y = projected_vertices.row(1).minCoeff();
    float max_x = projected_vertices.row(0).maxCoeff();
    float max_y = projected_vertices.row(1).maxCoeff();

    //Side length of each grid cell
    // Note: The side lengths can be zero if the axis is orthogonal to (1,1,1)
    float side_length_x = abs(x_axis.dot(this->cell_size));
    float side_length_y = abs(y_axis.dot(this->cell_size));

    //This is done to avoid issues with very small side lengths
    if(side_length_x < this->cell_size[0] / 10)
        side_length_x = this->cell_size[0];

    if(side_length_y < this->cell_size[1] / 10)
        side_length_y = this->cell_size[1];

    assert(side_length_x > 0);
    assert(side_length_y > 0);

    cout << "Will execute " << ((max_x-min_x)/side_length_x) * ((max_y-min_y)/side_length_y) << " point checks." << endl;

    //Sample points in the bounding box
    vector<Vector3f> points;
    for (float x = min_x; x <= max_x; x += side_length_x)
    {
        for (float y = min_y; y <= max_y; y += side_length_y)
        {
            //Check if point is inside triangle
            if (point_inside_triangle(Vector3f(x, y, 0), v0_proj, v1_proj, v2_proj))
            {
                //Here we are using the fact the the inverse is the transpose for a proper rotation matrix
                Vector3f unprojected_point = projection.transpose() * Vector3f(x, y, 0) + v0;
                points.push_back(unprojected_point);
            }
        }
    }
    return points;
}

/// @brief Check is the given point is in a cell that is already occupied.
/// @param point Query point, expressed in world coordinates.
/// @return Returns the index of the cell that contains the point if it is occupied, zero otherwise.
uint32_t OccupancyGrid::is_cell_occupied(const Vector3f& point){
    //Get cell index that would contain the point
    uint32_t cell_idx = this->idx_cell_at(point);
    if (cell_idx == 0){
        //The point is outside the grid
        return 0;
    }
    size_t exists = this->grid_cells.count(cell_idx);
    if (exists == 0){
        //The cell does not exist
        return 0;
    }
    if (exists > 1){
        throw runtime_error("There is more than one cell with the same index.");
    }
    //The cell exists and there is only one of them
    return cell_idx;
}

uint32_t OccupancyGrid::is_cell_occupied(uint32_t cell_idx)
{
    if (cell_idx == 0){
        //The point is outside the grid
        return 0;
    }
    size_t exists = this->grid_cells.count(cell_idx);
    if (exists == 0){
        //The cell does not exist
        return 0;
    }
    if (exists > 1){
        throw runtime_error("There is more than one cell with the same index.");
    }
    //The cell exists and there is only one of them
    return cell_idx;
}


/// @brief Returns the centre point of the cell at the given index.
/// @param idx The index of the cell to get the centre point of.
/// @return The centre point of the cell as a Vector3f.
Vector3f OccupancyGrid::cell_centre(uint32_t idx)
{
    //Get the coordinates of the cell
    vector<int> cell_coords = this->reverse_cell_idx(idx);
    //Get the position of the cell centre
    // The position in direction X is given by:
    //      origin + 
    //      number of cells in X direction * cell size in X direction +
    //      half of the cell size in X direction (to get the centre of the cell from its corner)
    Vector3f cell_centre = this->bb_origin + this->cell_size/2;
    cell_centre[0] += cell_coords[0]*this->cell_size[0];
    cell_centre[1] += cell_coords[1]*this->cell_size[1];
    cell_centre[2] += cell_coords[2]*this->cell_size[2];
    return cell_centre;
}

/// @brief Get the index of the cell that contains a given point.
/// @param point 3x1 vector representing the point.
/// @return Index of the cell that contains the point, or zero if the point is outside the grid.
uint32_t OccupancyGrid::idx_cell_at(const Vector3f& point)
{
    // Check if point is inside the bounding box of the occupancy grid
    if ((point[0] < this->bb_origin[0]) || (point[0] > this->bb_origin[0] + this->bb_extents[0] + this->cell_size[0]) ||
        (point[1] < this->bb_origin[1]) || (point[1] > this->bb_origin[1] + this->bb_extents[1] + this->cell_size[1]) ||
        (point[2] < this->bb_origin[2]) || (point[2] > this->bb_origin[2] + this->bb_extents[2] + this->cell_size[2]))
    {
        // Point is outside bounding box, there is no cell that could contain it
        return 0;
    }

    // Get cell indices. The (i,j,k)th cell is the one that is reached
    // when moving i cells in the x direction, j cells in the y direction,
    // and k cells in the z direction from the origin of the bounding box.
    int i = (point[0] - this->bb_origin[0]) / this->cell_size[0];
    int j = (point[1] - this->bb_origin[1]) / this->cell_size[1];
    int k = (point[2] - this->bb_origin[2]) / this->cell_size[2];

    //There is this->resolution cells along each dimension
    // We can assume that i, j, and k are positive as we checked that the point is inside the bounding box.
    assert(i >= 0);
    assert(j >= 0);
    assert(k >= 0);
    uint32_t idx = 1 + i + j*this->resolution + k*this->resolution*this->resolution;

    //cout << "Cell with indices (" << i << ", " << j << ", " << k << ") has index " << idx << endl;

    // Return cell indices
    return idx;
}

/// @brief Get the coordinates of the cell with the given index.
/// @param idx Index of the cell.
/// @return 3x1 vector representing the coordinates of the cell.
vector<int> OccupancyGrid::reverse_cell_idx(uint32_t idx)
{
    //Perform the reverse operation of idx_cell_at which is
    //  int idx = i + j*this->resolution + k*this->resolution*this->resolution;

    // Get cell indices
    idx -= 1;
    int i = idx % this->resolution;
    int j = (idx / this->resolution) % this->resolution;
    int k = idx / (this->resolution*this->resolution);

    //cout << "Cell with index " << idx << " has coordinates (" << i << ", " << j << ", " << k << ")" << endl;

    // Return cell coordinates
    return {i, j, k};
}

/// @brief  Get the list of grid cells.
/// @return List of grid cells.
unordered_map<uint32_t, GridCell>* OccupancyGrid::get_grid_cells()
{
    return &(this->grid_cells);
}

/// @brief Builds a grid of cells that represent the volume of the object.
/// @param vertices Array of vertices of the triangle mesh representing the object, one vertex per row.
/// @param triangles Array of indices of the triangle mesh representing the object, one triangle per row.
/// @param resolution Desired number of cells per unit of length.
OccupancyGrid::OccupancyGrid(const MatrixX3f& vertices, const MatrixX3i& triangles, int resolution)
{
    //The resolution is the number of cells along each dimension
    // and must be at least 1.
    assert(resolution > 0);

    // Get bounding box
    float min_x = vertices.col(0).minCoeff();
    float min_y = vertices.col(1).minCoeff();
    float min_z = vertices.col(2).minCoeff();
    float max_x = vertices.col(0).maxCoeff();
    float max_y = vertices.col(1).maxCoeff();
    float max_z = vertices.col(2).maxCoeff();

    // Get bounding box extents
    this->bb_extents = {max_x - min_x, max_y - min_y, max_z - min_z};

    // Get grid resolution, which is the number of cells along each dimension
    this->resolution = resolution;

    // Get grid cell size
    this->cell_size = {this->bb_extents[0] /(resolution-1), this->bb_extents[1]/(resolution-1), this->bb_extents[2] /(resolution-1)};

    // Get grid origin, slightly greater than the extent such the the mesh is inside the grid
    this->bb_origin = {min_x - this->cell_size[0]/2, min_y - this->cell_size[1]/2, min_z - this->cell_size[2]/2};

    // Iterate over triangles
    for (int i = 0; i < triangles.rows(); i++)
    {
        //cout << "Triangle " << i << endl;
        // Get triangle vertices
        Vector3f v0 = vertices.row(triangles(i, 0));
        Vector3f v1 = vertices.row(triangles(i, 1));
        Vector3f v2 = vertices.row(triangles(i, 2));

        // Get triangle normal assuming that the indices are
        // ordered with the PhysX convention.
        Vector3f normal = (v1 - v0).cross(v2 - v0).normalized();

        // Sample points in the triangle
        vector<Vector3f> sampled_points = sample_points_in_triangle(v0, v1, v2);

        //For each sample point, check if a grid cell has already been created for it
        // If not, create a new grid cell
        for (int j = 0; j < sampled_points.size(); j++){
            //Get the cell indices, will be zero for points outside the grid
            uint32_t cell_idx = this->idx_cell_at(sampled_points[j]);
            if(cell_idx != 0){
                //Create the oriented surface point
                shared_ptr<OrientedPoint> surface_point = make_shared<OrientedPoint>();
                surface_point->position = sampled_points[j];
                surface_point->normal   = normal;
                //Check if the cell is already occupied
                bool occupied = this->is_cell_occupied(cell_idx);
                if(!occupied){
                    //Get the centre of the cell
                    Vector3f cell_centre = this->cell_centre(cell_idx);
                    //Create the grid cell
                    GridCell grid_cell = GridCell(cell_idx, surface_point, this->cell_size, cell_centre);
                    //Add the grid cell to the list of grid cells
                    this->grid_cells.emplace(cell_idx, grid_cell);
                    //cout << "Index of cell containing point " << sampled_points[j].transpose() << " is " << cell_idx << endl;
                }else{
                    //Add the point to the existing grid cell
                    this->grid_cells.at(cell_idx).additional_point(surface_point);
                }
            }
        }
    }
}
        