//Define classes Point2D and Point3D by extending Vector2f and Vector3f respectively
// enabling their use in unordered_set and unordered_map by providing a hash function
// and overloading the comparison operators.
#pragma once

#include "AssemblyCD.h"
#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Point2D : public Vector2f
{
    private:
        float EPSILON = 1e-6;
    public:
        Point2D();
        Point2D(float x, float y);
        Point2D(const Vector2f& other);
        Point2D(const Point2D& other);
        bool operator==(const Point2D& other) const;
        struct HashFunction
        {
            size_t operator()(const Point2D& point) const
            {
                float approx_x = round(point[0] / point.EPSILON) * point.EPSILON;
                float approx_y = round(point[1] / point.EPSILON) * point.EPSILON;
                size_t xHash = hash<float>()(approx_x);
                size_t yHash = hash<float>()(approx_y) << 1;
                return xHash ^ yHash;
            }
        };
        friend struct EpsilonEquality2D;
};

struct EpsilonEquality2D
{
    bool operator()(const Point2D& lhs, const Point2D& rhs) const
    {
        float x_dist = abs(lhs[0] - rhs[0]);
        float y_dist = abs(lhs[1] - rhs[1]);
        return x_dist < lhs.EPSILON 
            && y_dist < lhs.EPSILON;
    }
};

class Point3D : public Vector3f
{
    private:
        float EPSILON = 1e-6;
    public:
        Point3D();
        Point3D(float x, float y, float z);
        Point3D(const Vector3f& other);
        Point3D(const Point3D& other);
        bool operator==(const Point3D& other) const;
        struct HashFunction
        {
            size_t operator()(const Point3D& point) const
            {
                float approx_x = round(point[0] / point.EPSILON) * point.EPSILON;
                float approx_y = round(point[1] / point.EPSILON) * point.EPSILON;
                float approx_z = round(point[2] / point.EPSILON) * point.EPSILON;
                size_t xHash = hash<float>()(approx_x);
                size_t yHash = hash<float>()(approx_y) << 1;
                size_t zHash = hash<float>()(approx_z) << 2;
                return (xHash ^ yHash) ^ zHash;
            }
        };
    friend struct EpsilonEquality3D;
};

struct EpsilonEquality3D
{
    bool operator()(const Point3D& lhs, const Point3D& rhs) const
    {
        return abs(lhs[0] - rhs[0]) < lhs.EPSILON 
            && abs(lhs[1] - rhs[1]) < lhs.EPSILON 
            && abs(lhs[2] - rhs[2]) < lhs.EPSILON;
    }
};