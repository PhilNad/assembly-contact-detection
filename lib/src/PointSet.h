#pragma once
//Define 2D and 3D point set classes such that a point set is an unordered set of Point2D and Point3D respectively.

#include <unordered_set>
#include <memory>
#include <eigen3/Eigen/Eigen>
#include "Points.h"

class PointSet2D : public std::unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>
{
    private:
        std::unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D> _pointset;
    public:
        PointSet2D();
        PointSet2D(const std::vector<Eigen::Vector2f>& points);
        PointSet2D(const std::unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>& points);
        PointSet2D(const PointSet2D& other);
        PointSet2D& operator=(const std::vector<Eigen::Vector2f>& points);
        std::pair<iterator, bool> insert(const Eigen::Vector2f& point);
        std::pair<iterator, bool> insert(const Point2D& point);
        std::pair<iterator, bool> insert(const PointSet2D& pointset);
        std::unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>::iterator begin();
        std::unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>::iterator end();
        std::unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>::const_iterator begin() const;
        std::unordered_set<Point2D, Point2D::HashFunction, EpsilonEquality2D>::const_iterator end() const;
        size_t size() const;
};


class PointSet3D : public std::unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>
{
    private:
        std::unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D> _pointset;
    public:
        PointSet3D();
        PointSet3D(const std::vector<Eigen::Vector3f>& points);
        PointSet3D(const std::unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>& points);
        PointSet3D(const PointSet3D& other);
        PointSet3D& operator=(const std::vector<Eigen::Vector3f>& points);
        std::pair<iterator, bool> insert(const Eigen::Vector3f& point);
        std::pair<iterator, bool> insert(const Point3D& point);
        std::pair<iterator, bool> insert(const PointSet3D& pointset);
        std::unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>::iterator begin();
        std::unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>::iterator end();
        std::unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>::const_iterator begin() const;
        std::unordered_set<Point3D, Point3D::HashFunction, EpsilonEquality3D>::const_iterator end() const;
        size_t size() const;
};