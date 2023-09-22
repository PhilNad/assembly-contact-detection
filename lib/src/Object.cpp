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
    vertices{vertices},
    triangles{triangles},
    is_fixed{is_fixed},
    mass{mass},
    com{com},
    material_name{material_name}
{}
Object::~Object(){}

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
MatrixX3f Object::get_vertices()
{
    return this->vertices;
}

/// @brief Get the triangles of the object
/// @return Mx3 matrix representing the triangles of the object
MatrixX3i Object::get_triangles()
{
    return this->triangles;
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
    this->vertices = vertices;
}

/// @brief Set the triangles of the object
/// @param triangles Mx3 matrix representing the triangles of the object
void Object::set_triangles(MatrixX3i triangles)
{
    this->triangles = triangles;
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

/// @brief Get the vertices of the object in world coordinates
/// @return Nx3 matrix representing the vertices of the object in world coordinates
MatrixX3f Object::get_world_vertices()
{
    Matrix3f R = this->pose.block<3, 3>(0, 0);
    Vector3f t = this->pose.block<3, 1>(0, 3);
    MatrixX3f vert_wrt_world = (R * this->vertices.transpose()).transpose().rowwise() + t.transpose();
    return vert_wrt_world;
}