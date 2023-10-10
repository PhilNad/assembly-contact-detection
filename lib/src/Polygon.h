#pragma once
#include <vector>
#include <eigen3/Eigen/Eigen>
#include "PointSet.h"
#include "Points.h"
#include "Triangle.h"
#include "AARectangle.h"

//A polygon is defined by a set of edges, each of which is a tuple of two points.
// See https://en.wikipedia.org/wiki/Convex_polygon for useful properties.
class Convex2DPolygon 
{
    private:
        std::vector<std::pair<Point2D, Point2D>> _edges;
        std::vector<Eigen::Vector2f> _edge_normals;
        std::vector<Triangle<Eigen::Vector2f>> _triangles;
        PointSet2D _vertices;
        Point2D _centroid;
        float _area = 0;
    public:
        Convex2DPolygon();
        Convex2DPolygon(const Triangle<Eigen::Vector2f>& triangle);
        Convex2DPolygon(AARectangle& rectangle);
        Convex2DPolygon(const PointSet2D& vertices);
        Convex2DPolygon(const std::vector<std::pair<Point2D, Point2D>> edges);
        std::vector<std::pair<Point2D, Point2D>> edges();
        PointSet2D vertices();
        bool contains(const Point2D& point);
        PointSet2D line_intersection(const Eigen::Vector2f& segment_p0, const Eigen::Vector2f& segment_p1);
        Convex2DPolygon polygon_intersection(Convex2DPolygon& polygon);
};