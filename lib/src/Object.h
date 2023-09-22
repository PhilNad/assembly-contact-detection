#pragma once

#include <vector>
#include <string>
#include <eigen3/Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Object
{
    private:
        Matrix4f pose;
        MatrixX3f vertices;
        MatrixX3i triangles;
        bool is_fixed;
        float mass;
        Vector3f com;
        string material_name;
    public:
        string id;
        Object(
            string /*id*/, 
            Matrix4f /*pose*/, 
            MatrixX3f /*vertices*/, 
            MatrixX3i /*triangles*/,
            bool /*is_fixed*/,
            float /*mass*/,
            Vector3f /*com*/,
            string /*material name*/
        );
        ~Object();
        string get_id();
        Matrix4f get_pose();
        MatrixX3f get_vertices();
        MatrixX3i get_triangles();
        bool get_is_fixed();
        float get_mass();
        Vector3f get_com();
        string get_material_name();
        void set_pose(Matrix4f /*pose*/);
        void set_vertices(MatrixX3f /*vertices*/);
        void set_triangles(MatrixX3i /*triangles*/);
        void set_is_fixed(bool /*is_fixed*/);
        void set_mass(float /*mass*/);
        void set_com(Vector3f /*com*/);
        void set_material_name(string /*material_name*/);
        MatrixX3f get_world_vertices();
};