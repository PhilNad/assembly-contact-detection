#include "Triangle.h"

template <>
void Triangle<Eigen::Vector2f>::compute_signed_area()
{
    //Compute vertices relative to first vertex
    Eigen::Vector2f v1r, v2r;
    v1r = vertex_1 - vertex_0;
    v2r = vertex_2 - vertex_0;

    //Compute signed area
    this->signed_area = (v1r[0]*v2r[1] - v1r[1]*v2r[0])/2; 
}

template <>
void Triangle<Eigen::Vector3f>::compute_signed_area()
{
    //Compute vertices relative to first vertex
    Eigen::Vector3f v1r, v2r;
    v1r = vertex_1 - vertex_0;
    v2r = vertex_2 - vertex_0;

    //Compute signed area
    this->signed_area = v1r.cross(v2r).norm()/2; 
}

template <typename T>
Triangle<T>::Triangle(T vertex_0, T vertex_1, T vertex_2) : vertex_0(vertex_0), vertex_1(vertex_1), vertex_2(vertex_2)
{
    //Compute the signed area of the triangle
    this->compute_signed_area();

    //If the signed area is negative, we swap the vertices
    // As a result, the vertices should be ordered counter-clockwise
    if(this->signed_area < 0){
        T temp = this->vertex_1;
        this->vertex_1 = this->vertex_2;
        this->vertex_2 = temp;
        this->signed_area = -this->signed_area;
    }
}

/// @brief Determines whether a given point is inside a triangle defined by three vertices.
/// @param point The point to check.
/// @param boundary_included If true, points on the triangle edges are considered to be inside the triangle.
/// @return True if the point is inside the triangle, false otherwise.
/// @note See: https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Barycentric_coordinates_on_triangles
/// @note See: Real-Time Collision Detection page 52.
template <typename T>
bool Triangle<T>::contains(const T& point, bool boundary_included)
{
    //Tolerance such that points directly on the triangle edge are considered to be inside the triangle
    float tol = 1e-6;
    if(boundary_included == false){
        tol = 0;
    }

    //Some computations can be performed only once and reused if the function is called multiple times for the same triangle
    if(pt_in_tri_cache.active == false){
        pt_in_tri_cache.v0r = vertex_1 - vertex_0;
        pt_in_tri_cache.v1r = vertex_2 - vertex_0;
        pt_in_tri_cache.d00 = pt_in_tri_cache.v0r.dot(pt_in_tri_cache.v0r);
        pt_in_tri_cache.d01 = pt_in_tri_cache.v0r.dot(pt_in_tri_cache.v1r);
        pt_in_tri_cache.d11 = pt_in_tri_cache.v1r.dot(pt_in_tri_cache.v1r);
        pt_in_tri_cache.inv_denom = 1/(pt_in_tri_cache.d00 * pt_in_tri_cache.d11 - pt_in_tri_cache.d01 * pt_in_tri_cache.d01);
        pt_in_tri_cache.active = true;
    }

    //Express points relative to first vertex
    T v0r, v1r, v2r;
    v0r = pt_in_tri_cache.v0r;
    v1r = pt_in_tri_cache.v1r;
    v2r = point - vertex_0;

    //Compute dot products
    float d00, d01, d11, d20, d21;
    d00 = pt_in_tri_cache.d00;
    d01 = pt_in_tri_cache.d01;
    d11 = pt_in_tri_cache.d11;
    d20 = v2r.dot(v0r);
    d21 = v2r.dot(v1r);

    //Inverse of the denominator
    float inv_denom;
    inv_denom = pt_in_tri_cache.inv_denom;

    //Barycentric weights that each describe the area of a sub-triangle over the area of the triangle
    // so they must be between 0 and 1. The weights also describe how much each vertex attracts the point.
    float alpha = (d11 * d20 - d01 * d21) * inv_denom;
    float beta  = (d00 * d21 - d01 * d20) * inv_denom;
    float gamma = 1.0f - alpha - beta;
    //cout << "alpha = " << alpha << ", beta = " << beta << ", gamma = " << gamma << endl;

    //Check if point is inside triangle
    return ((0 < alpha+tol) && (alpha-tol < 1) &&
            (0 < beta+tol)  && (beta-tol  < 1) &&
            (0 < gamma+tol) && (gamma-tol < 1));
}


//Template instanciations such that the linker can find the definitions
template class Triangle<Eigen::Vector2f>;
template class Triangle<Eigen::Vector3f>;


