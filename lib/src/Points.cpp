#include "AssemblyCD.h"

using namespace std;
using namespace Eigen;


Point2D::Point2D() : Vector2f(0.0f, 0.0f) {}
Point2D::Point2D(float x, float y) : Vector2f(x, y) {}
Point2D::Point2D(const Vector2f& other) : Vector2f(other) {}
Point2D::Point2D(const Point2D& other) : Vector2f(other) {}


Point3D::Point3D() : Vector3f(0.0f, 0.0f, 0.0f) {}
Point3D::Point3D(float x, float y, float z) : Vector3f(x, y, z) {}
Point3D::Point3D(const Vector3f& other) : Vector3f(other) {}
Point3D::Point3D(const Point3D& other) : Vector3f(other) {}

