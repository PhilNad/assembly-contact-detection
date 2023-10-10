#include "AARectangle.h"

AARectangle::AARectangle(physx::PxPlane plane, Eigen::Vector3f centre, Eigen::Vector3f half_extents)
: plane(plane), centre(centre), half_extents(half_extents)
{
    //The absolute value of the normal to the plane (plane.d) is either [0,0,1], [0,1,0], or [1,0,0]
    // We can choose orthogonal u and v vectors accordingly
    if(abs(abs(plane.n[0]) - 1) < this->EPSILON){
        //Normal is [1,0,0]
        this->u << 0, 0, 1;
        this->v << 0, 1, 0;
        this->area = 4*this->half_extents[1]*this->half_extents[2];
    }else 
    if(abs(abs(plane.n[1]) - 1) < this->EPSILON){
        //Normal is [0,1,0]
        this->u << 1, 0, 0;
        this->v << 0, 0, 1;
        this->area = 4*this->half_extents[0]*this->half_extents[2];
    }else
    if(abs(abs(plane.n[2]) - 1) < this->EPSILON){
        //Normal is [0,0,1]
        this->u << 1, 0, 0;
        this->v << 0, 1, 0;
        this->area = 4*this->half_extents[0]*this->half_extents[1];
    }else{
        //If the rectangle is Axis Aligned, this should never be reached
        // unless the PxPlane was wrongly defined.
        throw std::runtime_error("Invalid plane normal.");
    }

    //Extents in the local frame
    this->u_extents = Eigen::Vector2f(this->get_min_u(), this->get_max_u());
    this->v_extents = Eigen::Vector2f(this->get_min_v(), this->get_max_v());
}

/// @brief Project a point onto the plane of the AARectangle and clamp it to the rectangle boundaries.
/// @param point 3D coordinates of the point in the world frame.
/// @return 2D coordinates of the clamped point in the rectangle's local frame.
Eigen::Vector2f AARectangle::project_and_clamp(const Eigen::Vector3f& point)
{
    //Project the point onto the rectangle
    Eigen::Vector2f projected_point = this->project_point(point);
    //Clip the projected point to the rectangle boundaries
    Eigen::Vector2f clipped_point = this->inside_or_on(projected_point);
    return clipped_point;
}

/// @brief Project a point onto the plane of the AARectangle.
/// @param point The point to project.
/// @return The coordinates of the projected point in the rectangle's local frame.
Eigen::Vector2f AARectangle::project_point(const Eigen::Vector3f& point)
{
    //Compute the vector from the rectangle centre to the point
    Eigen::Vector3f point_vector = point - this->centre;
    //Project the point vector onto the u and v axes
    float u = point_vector.dot(this->u);
    float v = point_vector.dot(this->v);
    //Return the coordinates of the projected point
    return Eigen::Vector2f(u, v);
}

/// @brief Unproject a point from the plane of the AARectangle to the world frame.
/// @param point 2D coordinates of the point in the rectangle's local frame.
/// @return 3D coordinates of the unprojected point in the world frame.
Eigen::Vector3f AARectangle::unproject_point(const Eigen::Vector2f& point)
{
    //Compute the point vector in the rectangle's local frame
    Eigen::Vector3f point_vector = point[0]*this->u + point[1]*this->v;
    //Return the coordinates of the unprojected point
    return point_vector + this->centre;
}

/// @brief Check if the rectangle contains a given 2D point expressed in the rectangle's local frame.
/// @param point 2D coordinates of the point in the rectangle's local frame.
/// @return True if the point is inside the rectangle, false otherwise.
bool AARectangle::contains(const Eigen::Vector2f& point)
{
    float tol = 1e-6;
    //Check if the point is inside the rectangle
    float u = point[0];
    float v = point[1];

    if(u < this->u_extents[0] - tol || u > this->u_extents[1] + tol){
        return false;
    }
    if(v < this->v_extents[0] - tol || v > this->v_extents[1] + tol){
        return false;
    }
    return true;
}

/// @brief Check if the rectangle contains a given 3D point expressed in the world frame.
/// @param point 3D coordinates of the point in the world frame.
/// @return True if the point is inside the rectangle, false otherwise.
bool AARectangle::contains(const Eigen::Vector3f& point)
{
    //Project the point onto the rectangle
    Eigen::Vector2f projected_point = this->project_point(point);
    //Check if the projected point is inside the rectangle
    return this->contains(projected_point);
}

/// @brief Project a point onto the plane of the AARectangle and clamp it to the rectangle boundaries.
/// @param point 2D coordinates of the point in the rectangle's local frame.
/// @return 2D coordinates of the clamped point in the rectangle's local frame.
Eigen::Vector2f AARectangle::inside_or_on(const Eigen::Vector2f& point)
{
    //Check if the point is inside the rectangle
    if(this->contains(point)){
        return point;
    }
    //If the point is not inside the rectangle, we clamp the point to the rectangle boundaries
    float u = std::min(std::max(point[0], this->u_extents[0]), this->u_extents[1]);
    float v = std::min(std::max(point[1], this->v_extents[0]), this->v_extents[1]);
    //Return the coordinates of the clamped point
    return Eigen::Vector2f(u, v);
}

/// @brief Project a triangle onto the the plane of a AARectangle.
/// @param triangle The triangle to project.
/// @return The projected triangle.
Triangle<Eigen::Vector2f> AARectangle::project_triangle(const Triangle<Eigen::Vector3f>& triangle)
{
    //Project the vertices of the triangle onto the rectangle
    Eigen::Vector2f vertex_0 = this->project_point(triangle.vertex_0);
    Eigen::Vector2f vertex_1 = this->project_point(triangle.vertex_1);
    Eigen::Vector2f vertex_2 = this->project_point(triangle.vertex_2);
    //Create a new triangle
    Triangle<Eigen::Vector2f> projected_triangle(vertex_0, vertex_1, vertex_2);
    return projected_triangle;
}

/// @brief Return the coordinates of the origin of the rectangle in the world frame.
/// @return 3D coordinates of the origin of the rectangle in the world frame.
Eigen::Vector3f AARectangle::get_uv_origin_in_world()
{
    //Return the coordinates of the centre of the rectangle in the world frame
    return this->unproject_point(Eigen::Vector2f(0, 0));
}

/// @brief Get the minimal boundary of the rectangle along the u axis relative to the centre of the rectangle.
/// @return The minimal boundary of the rectangle along the u axis relative to the centre of the rectangle.
float AARectangle::get_min_u()
{
    //There are three possible cases:
    // 1. The normal is [1,0,0], in which case the minimal boundary is the one with the smallest x coordinate
    // 2. The normal is [0,1,0], in which case the minimal boundary is the one with the smallest y coordinate
    // 3. The normal is [0,0,1], in which case the minimal boundary is the one with the smallest z coordinate
    if(abs(abs(this->plane.n[0]) - 1) < this->EPSILON){
        //Normal is [1,0,0]
        return -this->half_extents[2];
    }else
    if(abs(abs(this->plane.n[1]) - 1) < this->EPSILON){
        //Normal is [0,1,0]
        return -this->half_extents[0];
    }else
    if(abs(abs(this->plane.n[2]) - 1) < this->EPSILON){
        //Normal is [0,0,1]
        return -this->half_extents[0];
    }else{
        //If the rectangle is Axis Aligned, this should never be reached
        // unless the PxPlane was wrongly defined.
        throw std::runtime_error("Invalid plane normal.");
    }
}

/// @brief Get the maximal boundary of the rectangle along the u axis relative to the centre of the rectangle.
/// @return The maximal boundary of the rectangle along the u axis relative to the centre of the rectangle.
float AARectangle::get_max_u()
{
    //There are three possible cases:
    // 1. The normal is [1,0,0], in which case the maximal boundary is the one with the largest x coordinate
    // 2. The normal is [0,1,0], in which case the maximal boundary is the one with the largest y coordinate
    // 3. The normal is [0,0,1], in which case the maximal boundary is the one with the largest z coordinate
    if(abs(abs(this->plane.n[0]) - 1) < this->EPSILON){
        //Normal is [1,0,0]
        return this->half_extents[2];
    }else
    if(abs(abs(this->plane.n[1]) - 1) < this->EPSILON){
        //Normal is [0,1,0]
        return this->half_extents[0];
    }else
    if(abs(abs(this->plane.n[2]) - 1) < this->EPSILON){
        //Normal is [0,0,1]
        return this->half_extents[0];
    }else{
        //If the rectangle is Axis Aligned, this should never be reached
        // unless the PxPlane was wrongly defined.
        throw std::runtime_error("Invalid plane normal.");
    }
}

/// @brief Get the minimal boundary of the rectangle along the v axis relative to the centre of the rectangle.
/// @return The minimal boundary of the rectangle along the v axis relative to the centre of the rectangle.
float AARectangle::get_min_v()
{
    //There are three possible cases:
    // 1. The normal is [1,0,0], in which case the minimal boundary is the one with the smallest y coordinate
    // 2. The normal is [0,1,0], in which case the minimal boundary is the one with the smallest z coordinate
    // 3. The normal is [0,0,1], in which case the minimal boundary is the one with the smallest x coordinate
    if(abs(abs(this->plane.n[0]) - 1) < this->EPSILON){
        //Normal is [1,0,0]
        return -this->half_extents[1];
    }else
    if(abs(abs(this->plane.n[1]) - 1) < this->EPSILON){
        //Normal is [0,1,0]
        return -this->half_extents[2];
    }else
    if(abs(abs(this->plane.n[2]) - 1) < this->EPSILON){
        //Normal is [0,0,1]
        return -this->half_extents[1];
    }else{
        //If the rectangle is Axis Aligned, this should never be reached
        // unless the PxPlane was wrongly defined.
        throw std::runtime_error("Invalid plane normal.");
    }
}

/// @brief Get the maximal boundary of the rectangle along the v axis relative to the centre of the rectangle.
/// @return The maximal boundary of the rectangle along the v axis relative to the centre of the rectangle.
float AARectangle::get_max_v()
{
    //There are three possible cases:
    // 1. The normal is [1,0,0], in which case the maximal boundary is the one with the largest y coordinate
    // 2. The normal is [0,1,0], in which case the maximal boundary is the one with the largest z coordinate
    // 3. The normal is [0,0,1], in which case the maximal boundary is the one with the largest x coordinate
    if(abs(abs(this->plane.n[0]) - 1) < this->EPSILON){
        //Normal is [1,0,0]
        return this->half_extents[1];
    }else
    if(abs(abs(this->plane.n[1]) - 1) < this->EPSILON){
        //Normal is [0,1,0]
        return this->half_extents[2];
    }else
    if(abs(abs(this->plane.n[2]) - 1) < this->EPSILON){
        //Normal is [0,0,1]
        return this->half_extents[1];
    }else{
        //If the rectangle is Axis Aligned, this should never be reached
        // unless the PxPlane was wrongly defined.
        throw std::runtime_error("Invalid plane normal.");
    }
}