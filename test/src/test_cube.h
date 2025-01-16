#pragma once 

#include "eigen3/Eigen/Eigen"

class Cube{
    public:
        MatrixX3f vertices;
        MatrixX3i triangles;
        Cube(float extent_x, float extent_y, float extent_z){
            float hex = abs(extent_x)/2;
            float hey = abs(extent_y)/2;
            float hez = abs(extent_z)/2;
            vertices = MatrixXf::Zero(8, 3);
            vertices.row(0) << -hex, -hey, -hez;
            vertices.row(1) << -hex,  hey, -hez;
            vertices.row(2) <<  hex,  hey, -hez;
            vertices.row(3) <<  hex, -hey, -hez;
            vertices.row(4) << -hex, -hey,  hez;
            vertices.row(5) << -hex,  hey,  hez;
            vertices.row(6) <<  hex,  hey,  hez;
            vertices.row(7) <<  hex, -hey,  hez;

            triangles = MatrixXi::Zero(12, 3);
            //Top
            triangles.row(0) << 7, 6, 4;
            triangles.row(1) << 4, 6, 5;
            //Bottom
            triangles.row(2) << 0, 2, 3;
            triangles.row(3) << 2, 0, 1;
            //Side X+
            triangles.row(4) << 3, 2, 7;
            triangles.row(5) << 7, 2, 6;
            //Side X-
            triangles.row(6) << 4, 1, 0;
            triangles.row(7) << 1, 4, 5;
            //Side Y+
            triangles.row(8) << 5, 2, 1;
            triangles.row(9) << 5, 6, 2;
            //Side Y-
            triangles.row(10) << 3, 4, 0;
            triangles.row(11) << 4, 3, 7;
        }
        ~Cube(){}
        void translate(float x, float y, float z){
            for(int i = 0; i < vertices.rows(); i++){
                vertices.row(i) = vertices.row(i) + RowVector3f(x, y, z);
            }
        }
        void scale(float scaling, RowVector3f centre){
            for(int i = 0; i < vertices.rows(); i++){
                vertices.row(i) = abs(scaling)*(vertices.row(i) - centre) + centre;
            }
        }
        void rotate(float angle, RowVector3f axis, RowVector3f centre){
            Matrix3f rotation_matrix;
            rotation_matrix = AngleAxisf(angle, axis.normalized());
            for(int i = 0; i < vertices.rows(); i++){
                vertices.row(i) = (rotation_matrix*(vertices.row(i) - centre).transpose()).transpose() + centre;
            }
        }
};