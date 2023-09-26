#include "Object.h"

/// @brief Define an object in the scene with the following properties
/// @param id unique identifier for the object
/// @param pose 4x4 matrix representing the pose of the object
/// @param vertices Nx3 matrix representing the vertices of the object
/// @param triangles Mx3 matrix representing the triangles of the object
/// @param is_fixed boolean representing whether the object is fixed in space (default: false)
/// @param mass mass of the object (default: 1)
/// @param com 3x1 vector representing the center of mass of the object (default: [0, 0, 0])
/// @param material_name name of the material of the object (default: wood)
Object::Object(string id, Matrix4f pose, MatrixX3f vertices, MatrixX3i triangles, 
        bool is_fixed = false, 
        float mass = 1.0f, 
        Vector3f com = (Vector3f() << 0.0f, 0.0f, 0.0f).finished(), 
        string material_name = "wood")
:
    id{id},
    pose{pose},
    tri_vertices{vertices},
    tri_triangles{triangles},
    is_fixed{is_fixed},
    mass{mass},
    com{com},
    material_name{material_name},
    max_separation{0.04f}
{}
Object::~Object(){}

/// @brief Record the description of the tetrahedral mesh that represents the volume of the object
/// @param vertices Vertices of the mesh
/// @param indices Indices of the mesh, referring to the vertices, 4 per tetrahedron
void Object::set_tetra_mesh(PxArray<PxVec3> vertices, PxArray<PxU32> indices)
{
    //Convert PxArray to Eigen Matrix
    this->tetra_vertices.resize(vertices.size(), 3);
    for (int i = 0; i < vertices.size(); i++) {
        this->tetra_vertices(i, 0) = vertices[i].x;
        this->tetra_vertices(i, 1) = vertices[i].y;
        this->tetra_vertices(i, 2) = vertices[i].z;
    }

    this->tetra_indices.resize(indices.size()/4, 4);
    for (int i = 0; i < indices.size()/4; i++) {
        this->tetra_indices(i, 0) = indices[i*4+0];
        this->tetra_indices(i, 1) = indices[i*4+1];
        this->tetra_indices(i, 2) = indices[i*4+2];
        this->tetra_indices(i, 3) = indices[i*4+3];
    }
}

/// @brief Record the description of the triangle mesh that represents the surface of the object
/// @param simpleTriMesh Triangle mesh
void Object::set_tri_mesh(PxSimpleTriangleMesh& simpleTriMesh)
{
    //Convert PxSimpleTriangleMesh to Eigen Matrix
    this->tri_vertices.resize(simpleTriMesh.points.count, 3);
    for (int i = 0; i < simpleTriMesh.points.count; i++) {
        PxVec3* data;
        data = (PxVec3*)(simpleTriMesh.points.data + i*sizeof(PxVec3));
        this->tri_vertices(i, 0) = data->x;
        this->tri_vertices(i, 1) = data->y;
        this->tri_vertices(i, 2) = data->z;
    }

    this->tri_triangles.resize(simpleTriMesh.triangles.count, 3);
    PxU32* data;
    data = (PxU32*)(simpleTriMesh.triangles.data);
    for (int i = 0; i < simpleTriMesh.triangles.count; i++) {
        this->tri_triangles(i, 0) = data[i*3+0];
        this->tri_triangles(i, 1) = data[i*3+1];
        this->tri_triangles(i, 2) = data[i*3+2];
    }
}

/// @brief Create a grid of cells that represent the volume of the object.
/// @param resolution Number of cells per unit of length.
/// @return Pointer to the occupancy grid
shared_ptr<OccupancyGrid> Object::create_occupancy_grid(int resolution)
{
    MatrixX3f vertices = this->tri_vertices;
    MatrixX3i triangles = this->tri_triangles;
    //Build the grid
    shared_ptr<OccupancyGrid> grid = make_shared<OccupancyGrid>(vertices, triangles, resolution);
    this->occupancy_grid = grid;
    return this->occupancy_grid;
}

/// @brief Get the position of the centre of each occupied cell in the occupancy grid
/// @return Nx3 matrix representing the centre positions
MatrixX3f Object::get_voxel_centres()
{
    std::unordered_map<uint32_t, GridCell>* grid = this->occupancy_grid->get_grid_cells();
    MatrixX3f cube_centres(grid->size(), 3);
    int i = 0;
    for (auto& it : *grid) {
        cube_centres(i, 0) = it.second.centre(0);
        cube_centres(i, 1) = it.second.centre(1);
        cube_centres(i, 2) = it.second.centre(2);
        i++;
    }
    return cube_centres;
}

/// @brief Get the side lengths of each occupied cell in the occupancy grid
/// @return 3x1 vector representing the side lengths
Vector3f Object::get_voxel_side_lengths()
{
    std::unordered_map<uint32_t, GridCell>* grid = this->occupancy_grid->get_grid_cells();
    //All cells have the same side lengths
    Vector3f cube_side_lengths = grid->begin()->second.half_extents * 2;
    return cube_side_lengths;
}

/// @brief Get the id of the object
/// @return id of the object
string Object::get_id()
{
    return this->id;
}

/// @brief Get the pose of the object
/// @return 4x4 matrix representing the pose of the object
Matrix4f Object::get_pose()
{
    return this->pose;
}

/// @brief Get the vertices of the object
/// @return Nx3 matrix representing the vertices of the object
MatrixX3f Object::get_tri_vertices()
{
    return this->tri_vertices;
}

/// @brief Get the triangles of the object
/// @return Mx3 matrix representing the triangles of the object
MatrixX3i Object::get_tri_triangles()
{
    return this->tri_triangles;
}

/// @brief Get the vertices of the tetrahedral mesh that represents the volume of the object
/// @return Nx3 matrix representing the vertices 
MatrixX3f Object::get_tetra_vertices()
{
    return this->tetra_vertices;
}

/// @brief Get the indices of the tetrahedral mesh that represents the volume of the object
/// @return Mx4 matrix representing the indices
MatrixX4i Object::get_tetra_indices()
{
    return this->tetra_indices;
}

/// @brief Get whether the object is fixed in space
/// @return boolean representing whether the object is fixed in space
bool Object::get_is_fixed()
{
    return this->is_fixed;
}

/// @brief Get the mass of the object
/// @return mass of the object
float Object::get_mass()
{
    return this->mass;
}

/// @brief Get the center of mass of the object
/// @return 3x1 vector representing the center of mass of the object
Vector3f Object::get_com()
{
    return this->com;
}

/// @brief Get the name of the material of the object
/// @return name of the material of the object
string Object::get_material_name()
{
    return this->material_name;
}

/// @brief Set the maximal distance from the object to a valid contact point
/// @param max_separation Distance in the object's units
void Object::set_max_separation(float max_separation)
{
    this->max_separation = abs(max_separation);
}

/// @brief Set the pose of the object
/// @param pose 4x4 matrix representing the pose of the object
void Object::set_pose(Matrix4f pose)
{
    this->pose = pose;
}

/// @brief Set the vertices of the object
/// @param vertices Nx3 matrix representing the vertices of the object
void Object::set_vertices(MatrixX3f vertices)
{
    this->tri_vertices = vertices;
}

/// @brief Set the triangles of the object
/// @param triangles Mx3 matrix representing the triangles of the object
void Object::set_triangles(MatrixX3i triangles)
{
    this->tri_triangles = triangles;
}

/// @brief Set whether the object is fixed in space
/// @param is_fixed boolean representing whether the object is fixed in space
void Object::set_is_fixed(bool is_fixed)
{
    this->is_fixed = is_fixed;
}

/// @brief Set the mass of the object
/// @param mass mass of the object
void Object::set_mass(float mass)
{
    this->mass = mass;
}

/// @brief Set the center of mass of the object
/// @param com 3x1 vector representing the center of mass of the object
void Object::set_com(Vector3f com)
{
    this->com = com;
}

/// @brief Set the name of the material of the object
/// @param material_name name of the material of the object
void Object::set_material_name(string material_name)
{
    this->material_name = material_name;
}

/// @brief Get the vertices of the triangle mesh in world coordinates
/// @return Nx3 matrix representing the vertices of the object in world coordinates
MatrixX3f Object::get_world_vertices()
{
    Matrix3f R = this->pose.block<3, 3>(0, 0);
    Vector3f t = this->pose.block<3, 1>(0, 3);
    MatrixX3f vert_wrt_world = (R * this->tri_vertices.transpose()).transpose().rowwise() + t.transpose();
    return vert_wrt_world;
}