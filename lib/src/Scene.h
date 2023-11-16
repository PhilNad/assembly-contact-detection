#pragma once

class Scene;

#include <vector>
#include <set>
#include <memory>
#include <iostream>
#include <string>
#include "PxPhysicsAPI.h"
#include <eigen3/Eigen/Eigen>
#include "Object.h"

using namespace std;
using namespace Eigen;

//Forward declaration
class Contact;

class Scene
{
    private:
        physx::PxSimulationEventCallback* gContactReportCallback = nullptr;
        vector<shared_ptr<Object>> object_ptrs;
        void startupPhysics();
        void cleanupPhysics();
        void clear_contacts();
        void create_object_shapes(Object* obj, Matrix4f pose, int resolution, bool is_volumetric, bool is_fixed, float mass, Vector3f com);
        PxRigidActor* get_actor(string name);
        vector<PxShape*> get_actor_shapes(PxRigidActor* actor);
        PxArray<PxShape*> make_tetmesh(Object* obj);
        PxArray<PxShape*> make_canary_spheres(Object* obj, PxArray<PxShape*> tetConvexShapes);
        PxShape* create_voxel_shape(GridCell* cell, Matrix4f obj_world_pose);
        Matrix3f tetra_local_frame(const vector<Vector3f>& vertices);
        VectorXd check_points_in_tetra(const Matrix3Xf& points, const Matrix3f& T_world_to_local, const Vector3f& origin);
        void points_in_tetrahedron(Matrix3Xf voxel_vertices_matrix, vector<vector<Vector3f>> tetra_vertices_list, int index_first_tet, int index_last_tet, VectorXd& inside_volume);
        vector<Contact> get_contact_points(string id1, string id2);
        vector<Contact> get_penetrating_contact_points(string id1, string id2);
        Object* get_object_by_id(string id);
        string get_contact_id_at_point(string id, Vector3f point);
    public:
        float max_distance_factor = 0.2;
        Scene();
        ~Scene();
        void add_volumetric_object(
            string id, 
            Matrix4f pose, 
            MatrixX3f tri_vertices, 
            MatrixX3i tri_indices,
            MatrixX3f tetra_vertices,
            MatrixX4i tetra_indices,
            MatrixX3f canary_sphere_positions,
            int resolution = 15,
            bool is_fixed = false,
            float mass = 1,
            Vector3f com = Vector3f::Zero(),
            string material_name = "wood"
            );
        void add_object(
            string id, 
            Matrix4f pose, 
            MatrixX3f tri_vertices, 
            MatrixX3i tri_indices,
            int resolution = 15,
            bool compute_volume = false,
            bool is_fixed = false,
            float mass = 1,
            Vector3f com = Vector3f::Zero(),
            string material_name = "wood"
        );
        void remove_object(string id);
        void set_max_distance_factor(float max_distance_factor);
        Matrix4f get_object_pose(string id);
        void set_object_pose(string id, Matrix4f pose);
        void step_simulation(float dt);
        set<string> get_contacted_objects(string target_object);
        MatrixX3f get_contact_points_positions(string id1, string id2);
        Vector3f get_closest_contact_point(string id1, string id2, const Vector3f point);
        MatrixX3f get_penetrating_contact_point_positions(string id1, string id2);
        MatrixX3f get_all_contact_points(string id);
        MatrixX3f get_all_penetrating_contact_points(string id);
        MatrixX3f get_contact_convex_hull(string id, int vertex_limit = 255);
        Matrix3f get_best_contact_triangle(string id, vector<Triangle<Vector3f>> triangles, bool stable);
        vector<pair<string, Vector3f>> get_three_most_stable_contact_points(string id, int hull_max_size = 255);
        Matrix3f get_other_two_most_stable_contact_points(string id, Vector3f first_contact_point);
        Matrix3f get_other_one_most_stable_contact_points(string id, Vector3f first_contact_point, Vector3f second_contact_point);
        void merge_similar_contact_points(float position_threshold, float normal_threshold);
        MatrixX3f get_tri_vertices(string id);
        MatrixX3i get_tri_triangles(string id);
        MatrixX3f get_tetra_vertices(string id);
        MatrixX4i get_tetra_indices(string id);
        MatrixX3f get_voxel_centres(string id);
        Vector3f get_voxel_side_lengths(string id);
        MatrixX3f get_canary_sphere_positions(string id);
        void set_tetra_mesh(string id, MatrixX3f vertices, MatrixX4i indices);
        void set_canary_sphere_positions(string id, MatrixX3f canary_sphere_positions);
        
};
