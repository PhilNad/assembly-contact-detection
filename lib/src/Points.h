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
    public:
        Point2D();
        Point2D(float x, float y);
        Point2D(const Vector2f& other);
        Point2D(const Point2D& other);
        struct HashFunction
        {
            size_t operator()(const Point2D& point) const
            {
                size_t xHash = hash<float>()(point[0]);
                size_t yHash = hash<float>()(point[1]) << 1;
                return xHash ^ yHash;
            }
        };
};

class Point3D : public Vector3f
{
    public:
        Point3D();
        Point3D(float x, float y, float z);
        Point3D(const Vector3f& other);
        Point3D(const Point3D& other);
        struct HashFunction
        {
            size_t operator()(const Point3D& point) const
            {
                size_t xHash = hash<float>()(point[0]);
                size_t yHash = hash<float>()(point[1]) << 1;
                size_t zHash = hash<float>()(point[2]) << 2;
                return xHash ^ yHash ^ zHash;
            }
        };
};