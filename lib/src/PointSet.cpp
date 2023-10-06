#include "AssemblyCD.h"

using namespace std;
using namespace Eigen;


PointSet2D::PointSet2D(){}
PointSet2D& PointSet2D::operator=(const vector<Vector2f>& points)
{
    _pointset.clear();
    for (auto& point : points)
    {
        _pointset.insert(Point2D(point));
    }
    return *this;
}
PointSet2D::PointSet2D(const vector<Vector2f>& points)
{
    *this = points;
}
PointSet2D::PointSet2D(const unordered_set<Point2D, Point2D::HashFunction>& points)
{
    _pointset.clear();
    for (auto& point : points)
    {
        _pointset.insert(Point2D(point));
    }
}
PointSet2D::PointSet2D(const PointSet2D& other)
{
    _pointset.clear();
    for (auto& point : other)
    {
        _pointset.insert(Point2D(point));
    }
}
pair<std::__detail::_Node_iterator<Point2D, true, true>, bool> PointSet2D::insert(const Vector2f& point)
{
    return _pointset.insert(Point2D(point));
}
unordered_set<Point2D, Point2D::HashFunction>::iterator PointSet2D::begin()
{
    return _pointset.begin();
}
unordered_set<Point2D, Point2D::HashFunction>::iterator PointSet2D::end()
{
    return _pointset.end();
}
unordered_set<Point2D, Point2D::HashFunction>::const_iterator PointSet2D::begin() const
{
    return _pointset.begin();
}
unordered_set<Point2D, Point2D::HashFunction>::const_iterator PointSet2D::end() const
{
    return _pointset.end();
}
size_t PointSet2D::size() const
{
    return _pointset.size();
}



PointSet3D::PointSet3D(){}
PointSet3D& PointSet3D::operator=(const vector<Vector3f>& points)
{
    _pointset.clear();
    for (auto& point : points)
    {
        _pointset.insert(Point3D(point));
    }
    return *this;
}
PointSet3D::PointSet3D(const vector<Vector3f>& points)
{
    *this = points;
}
PointSet3D::PointSet3D(const unordered_set<Point3D, Point3D::HashFunction>& points)
{
    _pointset.clear();
    for (auto& point : points)
    {
        _pointset.insert(Point3D(point));
    }
}
PointSet3D::PointSet3D(const PointSet3D& other)
{
    _pointset.clear();
    for (auto& point : other)
    {
        _pointset.insert(Point3D(point));
    }
}
pair<std::__detail::_Node_iterator<Point3D, true, true>, bool> PointSet3D::insert(const Vector3f& point)
{
    return _pointset.insert(Point3D(point));
}
unordered_set<Point3D, Point3D::HashFunction>::iterator PointSet3D::begin()
{
    return _pointset.begin();
}
unordered_set<Point3D, Point3D::HashFunction>::iterator PointSet3D::end()
{
    return _pointset.end();
}
unordered_set<Point3D, Point3D::HashFunction>::const_iterator PointSet3D::begin() const
{
    return _pointset.begin();
}
unordered_set<Point3D, Point3D::HashFunction>::const_iterator PointSet3D::end() const
{
    return _pointset.end();
}
size_t PointSet3D::size() const
{
    return _pointset.size();
}
