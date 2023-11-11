#pragma once
#include <eigen3/Eigen/Eigen>
#include "PointSet.h"

template <typename T>
class Triangle
{
    private:
        struct point_inside_triangle_computations_cache {
            bool active = false;
            T v0r;
            T v1r;
            float d00;
            float d01;
            float d11;
            float inv_denom;
        };
        point_inside_triangle_computations_cache pt_in_tri_cache;
        void compute_signed_area();
    public:
        T vertex_0;
        T vertex_1;
        T vertex_2;
        float signed_area = 0;
        Triangle(T vertex_0, T vertex_1, T vertex_2);
        bool contains(const T& point, bool boundary_included);
        float shortest_distance_to_plane(const Eigen::Vector3f& point);
        Eigen::Vector3f project_on_triangle(const Eigen::Vector3f& point);
        Eigen::Vector3f closest_point_in_triangle(Eigen::Vector3f p);
        int nb_points_inside(const PointSet2D& points);
        int nb_points_inside(const PointSet3D& points);
};

