#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include "Polygon.h"

using namespace Eigen;

/// @brief Default constructor representing a zero area polygon.
Convex2DPolygon::Convex2DPolygon()
{
    //Define an empty/zero polygon.
    _area = 0.0f;
}

/// @brief Build a representation of a 2D convex polygon from a triangle.
/// @param triangle The triangle.
Convex2DPolygon::Convex2DPolygon(const Triangle<Eigen::Vector2f>& triangle)
{
    std::vector<std::pair<Point2D, Point2D>> edges;

    edges.push_back(std::make_pair(Point2D(triangle.vertex_0), Point2D(triangle.vertex_1)));
    edges.push_back(std::make_pair(Point2D(triangle.vertex_1), Point2D(triangle.vertex_2)));
    edges.push_back(std::make_pair(Point2D(triangle.vertex_2), Point2D(triangle.vertex_0)));

    *this = Convex2DPolygon(edges);
}

/// @brief Build a representation of a 2D convex polygon from a rectangle.
/// @param rectangle The axis-aligned rectangle.
Convex2DPolygon::Convex2DPolygon(AARectangle& rectangle)
{
    //Make 4 lines segments for the rectangle
    // p0--p1, p1--p2, p2--p3, p3--p0
    float rect_u_extent = rectangle.get_max_u() - rectangle.get_min_u();
    float rect_v_extent = rectangle.get_max_v() - rectangle.get_min_v();
    Point2D rect_p0 = Point2D(-rect_u_extent/2, -rect_v_extent/2);
    Point2D rect_p2 = Point2D(rect_u_extent/2, rect_v_extent/2);
    Point2D rect_p1 = Point2D(rect_p2[0], rect_p0[1]);
    Point2D rect_p3 = Point2D(rect_p0[0], rect_p2[1]);

    std::vector<std::pair<Point2D, Point2D>> edges;
    edges.push_back(std::make_pair(rect_p0, rect_p1));
    edges.push_back(std::make_pair(rect_p1, rect_p2));
    edges.push_back(std::make_pair(rect_p2, rect_p3));
    edges.push_back(std::make_pair(rect_p3, rect_p0));

    *this = Convex2DPolygon(edges);
}

/// @brief Build a representation of a 2D convex polygon from a set of vertices.
/// @param vertices The vertices of the polygon, in no specific order.
/// @note For a convex polygon, the vertices are the convex hull.
Convex2DPolygon::Convex2DPolygon(const PointSet2D& vertices)
{
    //If there are less than 3 vertices, we cannot define a polygon.
    if(vertices.size() >= 3){

        //Define edges without assuming any order
        std::vector<std::pair<Point2D, Point2D>> edges;

        //Compute the centroid as the average of the vertices
        for (auto& vertex : vertices)
        {
            _vertices.insert(vertex);
            _centroid += vertex;
        }
        _centroid /= vertices.size();

        //Compute the angle of each vertex relative to the horizontal axis.
        // The vertices are sorted by increasing angle such that edges are defined
        // in a counter-clockwise order.
        std::vector<std::pair<float, Vector2f>> angles;
        for (auto& vertex : vertices)
        {
            Vector2f vertex_vector = vertex - _centroid;
            //Use atan2 to get the angle in the range [-pi, pi]
            float angle = atan2(vertex_vector[1], vertex_vector[0]);
            angles.push_back(std::make_pair(angle, vertex_vector + _centroid));
        }

        //Sort the vertices by increasing angle
        std::sort(angles.begin(), angles.end(), [](const std::pair<float, Vector2f>& a, const std::pair<float, Vector2f>& b) -> bool
        {
            return a.first < b.first;
        });

        //Create the edges
        for (int i = 0; i < angles.size(); i++)
        {
            int j = (i + 1) % angles.size();
            edges.push_back(std::make_pair(Point2D(angles[i].second), Point2D(angles[j].second)));
        }

        *this = Convex2DPolygon(edges);
    }
}

/// @brief Build a representation of a 2D convex polygon from a set of edges.
/// @param edges Each edge is a tuple of two 2D points.
Convex2DPolygon::Convex2DPolygon(const std::vector<std::pair<Point2D, Point2D>> edges)
{

    //Create the vertex PointSet
    for (auto& edge : edges)
    {
        _edges.push_back(edge);
        _vertices.insert(edge.first);
        _vertices.insert(edge.second);
    }

    //Verify that the polygon is closed.
    // We do not enforce the order of the edges, so we must make sure that each vertex
    // appears exactly twice in the set of edges. As a result, the size of the PointSet
    // will be equal to the number of edges.
    if (_vertices.size() != edges.size())
    {
        throw std::invalid_argument("The polygon is not closed.");
    }

    //Compute the centroid as the average of the vertices
    for (auto& vertex : _vertices)
    {
        _centroid += vertex;
    }
    _centroid /= _vertices.size();

    //For each edge, compute the normal vector pointing outward from the polygon
    // with a magnitude equal to the distance from the centroid to the edge.
    for (auto& edge : edges)
    {
        //Points relative to the centroid
        Vector2f vertex_centroid = edge.first - _centroid;
        Vector2f edge_vector     = (edge.first - edge.second).normalized();
        //The normal vector is the vector rejection of the centroid-to-edge vector
        // projected onto the edge vector.
        Vector2f projection = vertex_centroid.dot(edge_vector) * edge_vector;
        Vector2f normal = vertex_centroid - projection;

        _edge_normals.push_back(normal);
    }

    //A convex polygon can be decomposed into triangles by connecting each vertex to the centroid.
    // We use the centroid as a common vertex for all triangles.
    for (auto& edge : edges)
    {
        Vector2f v0 = {edge.first[0], edge.first[1]};
        Vector2f v1 = {edge.second[0], edge.second[1]};
        Vector2f c  = {_centroid[0], _centroid[1]};
        Triangle<Vector2f> side_triangle = Triangle<Eigen::Vector2f>(v0, v1, c);
        _triangles.push_back(side_triangle);
        _area += abs(side_triangle.signed_area);
    }

}

/// @brief Get the edges of the polygon as a vector of tuples of two 2D points.
/// @return Edge list.
std::vector<std::pair<Point2D, Point2D>> Convex2DPolygon::edges()
{
    return _edges;
}

/// @brief Get the vertices of the polygon as a 2D PointSet.
/// @return Point set.
PointSet2D Convex2DPolygon::vertices()
{
    return _vertices;
}

/// @brief Determines whether a given point is inside the polygon.
/// @param point The point to check.
/// @return True if the point is inside the polygon, false otherwise.
bool Convex2DPolygon::contains(const Point2D& point)
{
    //The point is inside the polygon if it is any triangles
    for (auto& triangle : _triangles){
        if (triangle.contains(point, true) == false){
            return false;
        }
    }
    return true;
}

/// @brief Find the intersection points (if any) between a line segment and the polygon.
/// @param segment_p0 Start point of the line segment.
/// @param segment_p1 End point of the line segment.
/// @return Point set with up to two points representing the intersection points.
/// @note See Real-Time Collision Detection page 198.
PointSet2D Convex2DPolygon::line_intersection(const Eigen::Vector2f& segment_p0, const Eigen::Vector2f& segment_p1)
{
    float tol = 1e-6;
    //Each side of the polygon defines a half-space that potentially "clips" the length of
    // the line segment lying inside the polygon. We start with the full line segment and
    // iteratively clip it with each side of the polygon. If the length of the line segment
    // becomes zero at some point, we stop the clipping process and return an empty set.
    float t_first = 0;
    float t_last  = 1;

    Vector2f direction_vector = segment_p1 - segment_p0;

    for(auto& edge_normal : this->_edge_normals){
        //d is the distance from the centroid to the edge/side of the polygon
        float d = edge_normal.norm();
        //The normal vector is pointing outward from the polygon
        Vector2f normalized_edge_normal = edge_normal/d;
        //The denominator is the magnitude of the projection of the direction vector onto the normal vector.
        // If it is zero, the line segment is parallel to the edge and we need to test whether the line segment
        //   is outside the half-space, in which case there is no intersection. 
        // Is it is negative, the segment and plane normal are somewhat opposed and the parameter t
        //   will be the one of the intersection point that is closest to the segment start point
        //   and t_last will be update.
        // If it is positive, the segment and plane normal are somewhat aligned and the parameter t
        //   will be the one of the intersection point that is closest to the segment end point.
        //   and t_first will be updated.
        float denom = normalized_edge_normal.dot(direction_vector);
        //Distance from the plane/side along its normal vector to the segment start point
        float dist = d - normalized_edge_normal.dot(segment_p0 - _centroid);
        if(abs(denom) < tol){
            //The line segment is parallel to the edge
            if (dist < -tol){
                //The line segment is outside the half-space defined by the edge
                // as we assume that the normals are outward.
                return PointSet2D();
            }
        }else{
            float t = dist / denom;
            if (denom < tol) {
                //The line segment is entering the half-space defined by the edge
                // Proceed to clip the start of the line segment.
                if (t > t_first)
                    t_first = t;
            } else {
                //The line segment is exiting the half-space defined by the edge
                // Proceed to clip the end of the line segment.
                if (t < t_last) 
                    t_last = t;
            }
            //If the length of the line segment becomes zero, there is no intersection.
            if (t_first > t_last) 
                return PointSet2D();
        }
    }

    //Compute the intersections at t_first and t_last
    Vector2f intersection_1 = segment_p0 + t_first * direction_vector;
    Vector2f intersection_2 = segment_p0 + t_last * direction_vector;
    PointSet2D intersection_points;
    intersection_points.insert(intersection_1);
    intersection_points.insert(intersection_2);
    return intersection_points;
}

/// @brief Compute the convex polygon resulting from the intersection of this polygon with another polygon.
/// @param polygon The other polygon.
/// @return The intersection polygon whose area is the intersection of the two polygons.
Convex2DPolygon Convex2DPolygon::polygon_intersection(Convex2DPolygon& polygon)
{
    //The intersection polygon is defined by the intersection of each side of the two polygons.
    // So we iterate over the edges of *this and compute the intersection points with the other
    // polygon using the line_intersection() method. 

    PointSet2D intersection_points;

    //Iterate over the edges of this polygon
    for(auto& edge : this->_edges){
        //Compute the intersection points between the edge and the other polygon
        PointSet2D line_intersection_points = polygon.line_intersection(edge.first, edge.second);
        //Append
        intersection_points.insert(line_intersection_points);
    }

    if(intersection_points.size() >= 3){
        //If there are at least 3 intersection points, we can build a new polygon from them.
        return Convex2DPolygon(intersection_points);
    }else{
        //If there are less than 3 intersection points, the polygons do not intersect.
        return Convex2DPolygon();
    }
}