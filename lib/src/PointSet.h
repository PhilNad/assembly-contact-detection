#pragma once
//Define 2D and 3D point set classes such that a point set is an unordered set of Point2D and Point3D respectively.

#include "AssemblyCD.h"
#include <unordered_set>
#include <memory>
#include <eigen3/Eigen/Eigen>

class PointSet2D : public unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>
{
    private:
        unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D> _pointset;
    public:
        PointSet2D();
        PointSet2D(const vector<Vector2f>& points);
        PointSet2D(const unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>& points);
        PointSet2D(const PointSet2D& other);
        PointSet2D& operator=(const vector<Vector2f>& points);
        pair<iterator, bool> insert(const Vector2f& point);
        pair<iterator, bool> insert(const Point2D& point);
        pair<iterator, bool> insert(const PointSet2D& pointset);
        unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>::iterator begin();
        unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>::iterator end();
        unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>::const_iterator begin() const;
        unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>::const_iterator end() const;
        size_t size() const;
};


class PointSet3D : public unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>
{
    private:
        unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D> _pointset;
    public:
        PointSet3D();
        PointSet3D(const vector<Vector3f>& points);
        PointSet3D(const unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>& points);
        PointSet3D(const PointSet3D& other);
        PointSet3D& operator=(const vector<Vector3f>& points);
        pair<iterator, bool> insert(const Vector3f& point);
        pair<iterator, bool> insert(const Point3D& point);
        pair<iterator, bool> insert(const PointSet3D& pointset);
        unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>::iterator begin();
        unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>::iterator end();
        unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>::const_iterator begin() const;
        unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>::const_iterator end() const;
        size_t size() const;
};