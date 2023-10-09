#pragma once
#include "PxPhysicsAPI.h"
#include <eigen3/Eigen/Eigen>
#include "Triangle.h"

class AARectangle
{
    private:
        Eigen::Vector3f u;
        Eigen::Vector3f v;
        float EPSILON = 1e-3;
    public:
        physx::PxPlane plane;
        Eigen::Vector3f centre;
        Eigen::Vector3f half_extents;
        float area;
        Eigen::Vector2f u_extents;
        Eigen::Vector2f v_extents;
        AARectangle(physx::PxPlane plane, Eigen::Vector3f centre, Eigen::Vector3f half_extents);
        Eigen::Vector2f project_point(const Eigen::Vector3f& point);
        Eigen::Vector3f unproject_point(const Eigen::Vector2f& point);
        Triangle<Eigen::Vector2f> project_triangle(const Triangle<Eigen::Vector3f>& triangle);
        bool contains(const Eigen::Vector3f& point);
        bool contains(const Eigen::Vector2f& point);
        Eigen::Vector2f project_and_clamp(const Eigen::Vector3f& point);
        Eigen::Vector2f inside_or_on(const Eigen::Vector2f& point);
        float get_min_u();
        float get_max_u();
        float get_min_v();
        float get_max_v();
};