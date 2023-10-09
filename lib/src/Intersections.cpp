#include <iostream>
#include "Intersections.h"


using namespace Eigen;
using namespace std;

PointSet3D triangle_overlap_over_AARectangle(AARectangle& aarec, std::shared_ptr<Triangle<Vector3f>> t1, std::shared_ptr<Triangle<Vector3f>> t2)
{
    cout << "  Triangle overlap over AARectangle" << endl;
    cout << "    T1 Vertices: " << "(" << t1->vertex_0[0] << ", " << t1->vertex_0[1] << ", " << t1->vertex_0[2] << "), "
                            << "(" << t1->vertex_1[0] << ", " << t1->vertex_1[1] << ", " << t1->vertex_1[2] << "), "
                            << "(" << t1->vertex_2[0] << ", " << t1->vertex_2[1] << ", " << t1->vertex_2[2] << ")" << endl;
    cout << "    T2 Vertices: " << "(" << t2->vertex_0[0] << ", " << t2->vertex_0[1] << ", " << t2->vertex_0[2] << "), "
                            << "(" << t2->vertex_1[0] << ", " << t2->vertex_1[1] << ", " << t2->vertex_1[2] << "), "
                            << "(" << t2->vertex_2[0] << ", " << t2->vertex_2[1] << ", " << t2->vertex_2[2] << ")" << endl;
    
    //Project the 3D triangles onto the interface plane.
    // Since the triangles are projected onto an Axis-Aligned rectangle,
    // at least one of the world coordinates of the triangle vertices will be zero.
    // such that we can now work in 2D.
    Vector2f t1_v0_proj = aarec.project_point(t1->vertex_0);
    Vector2f t1_v1_proj = aarec.project_point(t1->vertex_1);
    Vector2f t1_v2_proj = aarec.project_point(t1->vertex_2);
    Triangle<Vector2f> t1_proj = Triangle<Vector2f>(t1_v0_proj, t1_v1_proj, t1_v2_proj);

    Vector2f t2_v0_proj = aarec.project_point(t2->vertex_0);
    Vector2f t2_v1_proj = aarec.project_point(t2->vertex_1);
    Vector2f t2_v2_proj = aarec.project_point(t2->vertex_2);
    Triangle<Vector2f> t2_proj = Triangle<Vector2f>(t2_v0_proj, t2_v1_proj, t2_v2_proj);

    //TODO: If the signed_area of a triangle is zero, then it can be simplified to a single edge.
    //TODO: Cache Triangle overlaps to avoid recomputing them.
    //TODO: If the rectangle is inside the triangle, then the overlap is the rectangle itself.
    PointSet2D t1_2d_intersections;

    //For each edge of the first triangle, we compute the intersection points with the second triangle
    //First edge: v0-v1
    PointSet2D line_rect_intersections = line_AARectangle_intersection(t1_v0_proj, t1_v1_proj, aarec);
    if(line_rect_intersections.size() > 0){
        Vector2f line_start;
        Vector2f line_end;
        if(line_rect_intersections.size() == 1){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            line_end   = line_start;
        }
        if(line_rect_intersections.size() == 2){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            advance(it, 1);
            line_end   = *it;
        }
        PointSet2D e1_intersections = edge_triangle_intersection(line_start, line_end, t2_proj);
        t1_2d_intersections.insert(e1_intersections);
    }

    //Second edge: v1-v2
    line_rect_intersections =  line_AARectangle_intersection(t1_v1_proj, t1_v2_proj, aarec);
    if(line_rect_intersections.size() > 0){
        Vector2f line_start;
        Vector2f line_end;
        if(line_rect_intersections.size() == 1){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            line_end   = line_start;
        }
        if(line_rect_intersections.size() == 2){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            advance(it, 1);
            line_end   = *it;
        }
        PointSet2D e2_intersections = edge_triangle_intersection(line_start, line_end, t2_proj);
        t1_2d_intersections.insert(e2_intersections);
    }

    //Third edge: v2-v0
    line_rect_intersections =  line_AARectangle_intersection(t1_v2_proj, t1_v0_proj, aarec);
    if(line_rect_intersections.size() > 0){
        Vector2f line_start;
        Vector2f line_end;
        if(line_rect_intersections.size() == 1){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            line_end   = line_start;
        }
        if(line_rect_intersections.size() == 2){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            advance(it, 1);
            line_end   = *it;
        }
        PointSet2D e3_intersections = edge_triangle_intersection(line_start, line_end, t2_proj);
        t1_2d_intersections.insert(e3_intersections);
    }

    //Verify that the overlap between the first triangle and the rectangle is significant
    if(t1_2d_intersections.size() >= 3){
        //Compute the area of overlap
        cout << "    Triangle 1 overlaps the rectangle with " << t1_2d_intersections.size() << " intersection points." << endl;
    }

    PointSet2D t2_2d_intersections;

    //For each edge of the second triangle, we compute the intersection points with the first triangle
    //First edge: v0-v1
    line_rect_intersections =  line_AARectangle_intersection(t2_v0_proj, t2_v1_proj, aarec);
    if(line_rect_intersections.size() > 0){
        Vector2f line_start;
        Vector2f line_end;
        if(line_rect_intersections.size() == 1){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            line_end   = line_start;
        }
        if(line_rect_intersections.size() == 2){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            advance(it, 1);
            line_end   = *it;
        }
        PointSet2D e4_intersections = edge_triangle_intersection(line_start, line_end, t1_proj);
        t2_2d_intersections.insert(e4_intersections);
    }

    //Second edge: v1-v2
    line_rect_intersections = line_AARectangle_intersection(t2_v1_proj, t2_v2_proj, aarec);
    if(line_rect_intersections.size() > 0){
        Vector2f line_start;
        Vector2f line_end;
        if(line_rect_intersections.size() == 1){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            line_end   = line_start;
        }
        if(line_rect_intersections.size() == 2){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            advance(it, 1);
            line_end   = *it;
        }
        PointSet2D e5_intersections = edge_triangle_intersection(line_start, line_end, t1_proj);
        t2_2d_intersections.insert(e5_intersections);
    }

    //Third edge: v2-v0
    line_rect_intersections =  line_AARectangle_intersection(t2_v2_proj, t2_v0_proj, aarec);
    if(line_rect_intersections.size() > 0){
        Vector2f line_start;
        Vector2f line_end;
        if(line_rect_intersections.size() == 1){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            line_end   = line_start;
        }
        if(line_rect_intersections.size() == 2){
            auto it = line_rect_intersections.begin();
            line_start = *it;
            advance(it, 1);
            line_end   = *it;
        }
        PointSet2D e6_intersections = edge_triangle_intersection(line_start, line_end, t1_proj);
        t2_2d_intersections.insert(e6_intersections);
    }

    //Unproject all points and append them to a list
    cout << "    Triangle 2 overlaps the rectangle with " << t2_2d_intersections.size() << " intersection points." << endl;
    PointSet2D all_2d_intersections;
    all_2d_intersections.insert(t1_2d_intersections);
    all_2d_intersections.insert(t2_2d_intersections);
    PointSet3D all_3d_intersections;
    for(auto& pt_2d : all_2d_intersections){
        Vector3f p3d = aarec.unproject_point(pt_2d);
        all_3d_intersections.insert(p3d);
    }
    for(auto& pt_3d : all_3d_intersections){
        cout << "  Intersection point: (" << pt_3d[0] << ", " << pt_3d[1] << ", " << pt_3d[2] << ")" << endl;
    }

    return all_3d_intersections;
}

/// @brief Find up to two intersection points between a line segment and a rectangle in 2D.
/// @param segment_p0 Start point of the line segment.
/// @param segment_p1 End point of the line segment.
/// @param rectangle Axis-Aligned rectangle.
/// @return List of up to two intersection points between the line segment and the rectangle.
PointSet2D line_AARectangle_intersection(const Vector2f& segment_p0, const Vector2f& segment_p1, AARectangle& rectangle)
{
    //Make 4 lines segments for the rectangle
    // p0--p1, p1--p2, p2--p3, p3--p0
    float rect_u_extent = rectangle.get_max_u() - rectangle.get_min_u();
    float rect_v_extent = rectangle.get_max_v() - rectangle.get_min_v();
    Vector2f rect_p0 = Vector2f(-rect_u_extent/2, -rect_v_extent/2);
    Vector2f rect_p2 = Vector2f(rect_u_extent/2, rect_v_extent/2);
    Vector2f rect_p1 = Vector2f(rect_p2[0], rect_p0[1]);
    Vector2f rect_p3 = Vector2f(rect_p0[0], rect_p2[1]);

    //Find up to two intersection points between the line segment and the rectangle edges
    PointSet2D intersections;
    int accumulated_nb_intersections = 0;
    //First side: p0--p1
    LineSegmentIntersection result = line_segment_intersection(segment_p0, segment_p1, rect_p0, rect_p1, true);
    accumulated_nb_intersections += result.nb_intersections;
    if(result.nb_intersections == 1){
        intersections.insert(result.intersection_point_1);
    }else if(result.nb_intersections == 2){
        //If the segment overlaps the rectangle edge, we will get two points.
        intersections.insert(result.intersection_point_1);
        intersections.insert(result.intersection_point_2);
    }
    //Second side: p1--p2
    result = line_segment_intersection(segment_p0, segment_p1, rect_p1, rect_p2, true);
    accumulated_nb_intersections += result.nb_intersections;
    if(result.nb_intersections == 1){
        intersections.insert(result.intersection_point_1);
    }else if(result.nb_intersections == 2){
        intersections.insert(result.intersection_point_1);
        intersections.insert(result.intersection_point_2);
    }
    //Third side: p2--p3
    if(accumulated_nb_intersections < 2){
        result = line_segment_intersection(segment_p0, segment_p1, rect_p2, rect_p3, true);
        accumulated_nb_intersections += result.nb_intersections;
        if(result.nb_intersections == 1){
            intersections.insert(result.intersection_point_1);
        }else if(result.nb_intersections == 2){
            intersections.insert(result.intersection_point_1);
            intersections.insert(result.intersection_point_2);
        }
    }
    //Fourth side: p3--p0
    if(accumulated_nb_intersections < 2){
        result = line_segment_intersection(segment_p0, segment_p1, rect_p3, rect_p0, true);
        accumulated_nb_intersections += result.nb_intersections;
        if(result.nb_intersections == 1){
            intersections.insert(result.intersection_point_1);
        }else if(result.nb_intersections == 2){
            intersections.insert(result.intersection_point_1);
            intersections.insert(result.intersection_point_2);
        }
    }

    //If there are less than 2 intersection points, we test if the edge endpoints are inside the rectangle
    if(accumulated_nb_intersections < 2){
        //Build two triangles from the rectangle
        Triangle<Vector2f> triangle1(rect_p0, rect_p1, rect_p2);
        Triangle<Vector2f> triangle2(rect_p0, rect_p2, rect_p3);
        //Test if the segment endpoints are inside the first half of the rectangle
        if(triangle1.contains(segment_p0, true)){
            intersections.insert(segment_p0);
            accumulated_nb_intersections++;
        }
        if(accumulated_nb_intersections < 2 && triangle1.contains(segment_p1, true)){
            intersections.insert(segment_p1);
            accumulated_nb_intersections++;
        }
        //Test if the segment endpoints are inside the second half of the rectangle
        if(accumulated_nb_intersections < 2 && triangle2.contains(segment_p0, true)){
            intersections.insert(segment_p0);
            accumulated_nb_intersections++;
        }
        if(accumulated_nb_intersections < 2 && triangle2.contains(segment_p1, true)){
            intersections.insert(segment_p1);
            accumulated_nb_intersections++;
        }
    }

    return intersections;
}

/// @brief Find up to two intersection points between an edge/segment and a triangle in 2D.
/// @param edge_p0 Start point of the edge/segment.
/// @param edge_p1 End point of the edge/segment.
/// @param triangle Two dimensional triangle.
/// @return Positions of up to two intersection points between the edge/segment and the triangle.
PointSet2D edge_triangle_intersection(const Vector2f& edge_p0, const Vector2f& edge_p1, Triangle<Vector2f>& triangle)
{
    //Each triangle edge will results in up to 2 points:
    // Case A - 0 point: the edge does not intersect the triangle
    // Case B - 1 point: a vertex of the edge is on the boundary of the triangle
    // Case C - 2 points: the edge crosses a segment of the triangle and reaches a point inside the triangle
    // Case D - 2 points: the edge goes in and out of the triangle, crossing two segments
    // Case E - 2 points: the edge is collinear with a segment of the triangle, and the two points are the endpoints of the intersection

    //If any edge has accumulated 2 intersection points (Case D and E), there is no need to test other edges from the same triangle.
    //After having tested all edges, the vertices of edges having accumulated less than 2 intersection points can be tested to see 
    // if they lie inside the triangle.

    PointSet2D intersections;
    int accumulated_nb_intersections = 0;

    //First triangle side
    LineSegmentIntersection result = line_segment_intersection(edge_p0, edge_p1, triangle.vertex_0, triangle.vertex_1, false);
    accumulated_nb_intersections += result.nb_intersections;
    if(result.nb_intersections == 1){
        intersections.insert(result.intersection_point_1);
    }else if(result.nb_intersections == 2){
        intersections.insert(result.intersection_point_1);
        intersections.insert(result.intersection_point_2);
    }
    //Second triangle side
    if(accumulated_nb_intersections < 2){
        result = line_segment_intersection(edge_p0, edge_p1, triangle.vertex_1, triangle.vertex_2, false);
        accumulated_nb_intersections += result.nb_intersections;
        if(result.nb_intersections == 1){
            intersections.insert(result.intersection_point_1);
        }else if(result.nb_intersections == 2){
            intersections.insert(result.intersection_point_1);
            intersections.insert(result.intersection_point_2);
        }
    }
    //Third triangle side
    if(accumulated_nb_intersections < 2){
        result = line_segment_intersection(edge_p0, edge_p1, triangle.vertex_2, triangle.vertex_0, false);
        accumulated_nb_intersections += result.nb_intersections;
        if(result.nb_intersections == 1){
            intersections.insert(result.intersection_point_1);
        }else if(result.nb_intersections == 2){
            intersections.insert(result.intersection_point_1);
            intersections.insert(result.intersection_point_2);
        }
    }
    //If there are less than 2 intersection points, we test if the edge endpoints are inside the triangle
    // to account for cases A, B, C.
    if(accumulated_nb_intersections < 2){
        if(triangle.contains(edge_p0, true)){
            intersections.insert(edge_p0);
            accumulated_nb_intersections++;
        }
        if(triangle.contains(edge_p1, true)){
            intersections.insert(edge_p1);
            accumulated_nb_intersections++;
        }
    }
    return intersections;
}

/// @brief Computes the intersection of two line segments.
/// @param p1 Start point (2D) of line segment 1.
/// @param q1 End point (2D) of line segment 1.
/// @param p2 Start point (2D) of line segment 2.
/// @param q2 End point (2D) of line segment 2.
/// @param closed_segments If true, the line segments are considered closed and the intersection points can be on the endpoints.
/// @return LineSegmentIntersection struct containing the intersection points and a boolean indicating if the line segments intersect.
LineSegmentIntersection line_segment_intersection(Vector2f p1, Vector2f q1, Vector2f p2, Vector2f q2, bool closed_segments)
{
    float tol = 1e-6;
    if(!closed_segments)
        tol = 0;
    //Each line can be expressed as a parametric equation
    // r = p + t*(q-p)
    // where (p, q) are two points on the line and t is a scalar.
    // The intersection point is the point for which the two parametric equations are equal:
    //  p1 + s*(q1-p1) = p2 + t*(q2-p2)
    // which can be rewritten as
    //  t*(q2-p2) - s*(q1-p1) = p1 - p2
    // and simplified with
    //  c = p1 - p2
    //  d1 = q1 - p1
    //  d2 = q2 - p2
    // producing 
    //  t*d2 - s*d1 = c
    // whose the solution obtained via Cramer's rule
    //  t = (d1[0] * c[1] - d1[1] * c[0]) / det
    //  s = (d2[0] * c[1] - d2[1] * c[0]) / det
    // with det=(a1*b2 - a2*b1) being the determinant of the system
    // that will be zero if the lines are parallel.
    // Relevant: https://stackoverflow.com/a/565282
    Vector2f d1 = q1 - p1;
    Vector2f d2 = q2 - p2;
    Vector2f c  = p1 - p2;
    //Norms
    float d1_norm = d1.norm();
    float d2_norm = d2.norm();
    float c_norm  = c.norm();
    //Numerators
    float num_t = (d1[0] * c[1] - d1[1] * c[0]);
    float num_s = (d2[0] * c[1] - d2[1] * c[0]);
    //Denominator / determinant
    float det = (d1[0] * d2[1] - d2[0] * d1[1]);

    struct LineSegmentIntersection result;

    //Dealing with degenerate cases (a line segment is a point)
    //If d1 == 0, then p1 == q1 and the first line segment is a point
    if(d1_norm == 0 && d2_norm > 0){
        //Check if the point lies on the second line segment
        // That is if c=p1-p2 is parallel to d2=q2-p2
        float s = c.dot(d2)/(c_norm*d2_norm);
        if(s > 1-1e-6 && c_norm < d2_norm+1e-6){
            //Return the intersection point
            result.nb_intersections = 1;
            result.intersection_point_1 = p1;
            return result;
        }else{
            //No intersection
            return result;
        }
    }
    //If d2 == 0, then p2 == q2 and the second line segment is a point.
    if(d2_norm == 0 && d1_norm > 0){
        //Check if the point lies on the first line segment
        // That is if c=p1-p2 is parallel to d1=q1-p1
        float s = c.dot(d1)/(c_norm*d1_norm);
        if(s > 1-1e-6 && c_norm < d1_norm+1e-6){
            //Return the intersection point
            result.nb_intersections = 1;
            result.intersection_point_1 = p2;
            return result;
        }else{
            //No intersection
            return result;
        }
    }
    //If both d1 == 0 and d2 == 0, check if the two points are the same,
    // in which case the intersetion is the point. Otherwise there is no intersection.
    if(d1_norm == 0 && d2_norm == 0){
        //Check if the two points are the same
        if(c_norm < 1e-6){
            //Return the intersection point
            result.nb_intersections = 1;
            result.intersection_point_1 = p1;
            return result;
        }else{
            //No intersection
            return result;
        }
    }

    //By here, we know that the two line segments are not degenerate.

    //If both the numerator and denominator are zero,
    // then the lines are collinear.
    if( abs(det)   < 1e-6 && 
        abs(num_s) < 1e-6 && 
        abs(num_t) < 1e-6){
        //We compute the overlap between the two line segments (possibly zero)
        float d12 = d1.dot(d2);
        float d11 = d1.dot(d1);
        float s1 = d1.dot(p2 - p1) / d11;
        float s2 = s1 + d12 / d11;

        //If d1 and d2 point in opposite directions
        // we swap s1 and s2
        if(d12 < 0){
            float tmp = s1;
            s1 = s2;
            s2 = tmp;
        }

        if(s1+tol > 1 || s2-tol < 0){
            //Lines are collinear but disjoint
            return result;
        }

        float s_min = max(0.0f, s1);
        float s_max = min(1.0f, s2);
        
        //Line segments overlap from s_min to s_max
        struct LineSegmentIntersection result;
        result.nb_intersections = 2;
        result.intersection_point_1 = p1 + s_min*d1;
        result.intersection_point_2 = p1 + s_max*d1;
        return result;
    }

    //If the denominator is zero but the numerators are not,
    // then the lines are parallel and non-intersecting.
    if(abs(det)   < 1e-6 && 
        (abs(num_s) > 1e-6 || abs(num_t) > 1e-6)){
        //Lines are parallel
        return result;
    }

    //Otherwise, there is a unique solution given by
    float s = num_s / det;
    float t = num_t / det;

    //If the intersection happens when a parameter is between 0 and 1,
    // then it means that the intersection happens within the line segment.
    if (s+tol > 0 && s-tol < 1 && t+tol > 0 && t-tol < 1){
        //Return the intersection point
        result.nb_intersections = 1;
        result.intersection_point_1 = p1 + (s * d1);
    }

    //The intersection happens outside of the line segment
    return result;
}