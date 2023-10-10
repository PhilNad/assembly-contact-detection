#pragma once
#include <memory>
#include "Triangle.h"
#include "AARectangle.h"
#include "PointSet.h"
#include "Polygon.h"


struct LineSegmentIntersection {
    Eigen::Vector2f intersection_point_1 = Eigen::Vector2f(NAN, NAN);
    Eigen::Vector2f intersection_point_2 = Eigen::Vector2f(NAN, NAN);
    int nb_intersections = 0;
};

PointSet3D triangle_triangle_AARectangle_intersection(AARectangle& aarec, std::shared_ptr<Triangle<Eigen::Vector3f>> t1, std::shared_ptr<Triangle<Eigen::Vector3f>> t2);

PointSet3D triangle_overlap_over_AARectangle(AARectangle& aarec, std::shared_ptr<Triangle<Eigen::Vector3f>> t1, std::shared_ptr<Triangle<Eigen::Vector3f>> t2);

PointSet2D line_AARectangle_intersection(const Eigen::Vector2f& segment_p0, const Eigen::Vector2f& segment_p1, AARectangle& rectangle);

PointSet2D edge_triangle_intersection(const Eigen::Vector2f& edge_p0, const Eigen::Vector2f& edge_p1, Triangle<Eigen::Vector2f>& triangle);

LineSegmentIntersection line_segment_intersection(Eigen::Vector2f p1, Eigen::Vector2f q1, Eigen::Vector2f p2, Eigen::Vector2f q2, bool closed_segments = true);

bool AARectangle_in_triangle(AARectangle& aarec, Triangle<Eigen::Vector2f>& triangle);

LineSegmentIntersection line_segment_ConvexPolygon_intersection(Eigen::Vector2f p1, Eigen::Vector2f q1, Convex2DPolygon& polygon, bool closed_segments = true);