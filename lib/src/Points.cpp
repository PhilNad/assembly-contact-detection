#include "AssemblyCD.h"

using namespace std;
using namespace Eigen;


Point2D::Point2D() : Vector2f(0.0f, 0.0f) {}
Point2D::Point2D(float x, float y) : Vector2f(x, y) {}
Point2D::Point2D(const Vector2f& other) : Vector2f(other) {}
Point2D::Point2D(const Point2D& other) : Vector2f(other) {}
bool Point2D::operator==(const Point2D& other) const
{
    return abs((*this)[0] - other[0]) < EPSILON
        && abs((*this)[1] - other[1]) < EPSILON;
}


Point3D::Point3D() : Vector3f(0.0f, 0.0f, 0.0f) {}
Point3D::Point3D(float x, float y, float z) : Vector3f(x, y, z) {}
Point3D::Point3D(const Vector3f& other) : Vector3f(other) {}
Point3D::Point3D(const Point3D& other) : Vector3f(other), normal(other.normal) {}
Point3D::Point3D(const Vector3f& other, const Vector3f& normal) : Vector3f(other), normal(normal) {}
bool Point3D::operator==(const Point3D& other) const
{
    return abs((*this)[0] - other[0]) < EPSILON
        && abs((*this)[1] - other[1]) < EPSILON
        && abs((*this)[2] - other[2]) < EPSILON;
}

