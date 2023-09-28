#include "Scene.h"
#include "PxPhysicsAPI.h"
#include <iostream>
#include <vector>
#include <set>
#include "eigen3/Eigen/Eigen"

using namespace physx;
using namespace std;

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
            // //Top
            // triangles.row(0) << 4, 6, 7;
            // triangles.row(1) << 4, 6, 5;
            // //Bottom
            // triangles.row(2) << 0, 2, 3;
            // triangles.row(3) << 0, 2, 1;
            // //Side X+
            // triangles.row(4) << 7, 2, 3;
            // triangles.row(5) << 7, 2, 6;
            // //Side X-
            // triangles.row(6) << 4, 1, 0;
            // triangles.row(7) << 4, 1, 5;
            // //Side Y+
            // triangles.row(8) << 5, 2, 1;
            // triangles.row(9) << 5, 2, 6;
            // //Side Y-
            // triangles.row(10) << 4, 3, 0;
            // triangles.row(11) << 4, 3, 7;

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

            // triangles.row(0) << 7, 4, 0;
            // triangles.row(1) << 1, 5, 6;
            // triangles.row(2) << 2, 6, 7;
            // triangles.row(3) << 6, 5, 4;
            // triangles.row(4) << 4, 5, 1;
            // triangles.row(5) << 3, 0, 1;
            // triangles.row(6) << 3, 7, 0;
            // triangles.row(7) << 2, 1, 6;
            // triangles.row(8) << 3, 2, 7;
            // triangles.row(9) << 7, 6, 4;
            // triangles.row(10) << 0, 4, 1;
            // triangles.row(11) << 2, 3, 1;
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
};


int main(int argc, char** argv) {
    // Initialize the scene
    Scene scene;

    // Add a cube to the scene
    string id = "cube1";
    Matrix4f pose;
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube cube1 = Cube(1, 1, 1);
    cube1.translate(pose(0, 3), pose(1, 3), pose(2, 3));
    bool is_fixed = false;
    float mass = 1.0f;
    Vector3f com = Vector3f::Zero();
    string material_name = "wood";
    scene.add_object(id, pose, cube1.vertices, cube1.triangles, is_fixed, mass, com, material_name);

    // Add another cube to the scene
    id = "cube2";
    pose << 1, 0, 0, 0,
            0, 1, 0, 0.25,
            0, 0, 1, 1.5,
            0, 0, 0, 1;
    Cube cube2 = Cube(1, 1, 1);
    cube2.translate(pose(0, 3), pose(1, 3), pose(2, 3));
    is_fixed = false;
    mass = 1.0f;
    com = Vector3f::Zero();
    material_name = "wood";
    scene.add_object(id, pose, cube2.vertices, cube2.triangles, is_fixed, mass, com, material_name);

    // Step the simulation
    float dt = 0.01f;
    scene.step_simulation(dt);

    // Get the list of objects in contact with the cube
    set<string> contacted_objects = scene.get_contacted_objects(id);
    cout << "Contacted objects: ";
    for(auto it = contacted_objects.begin(); it != contacted_objects.end(); ++it){
        cout << *it << " ";
    }
    cout << endl;

    // Get the contact points between the two objects
    MatrixX3f contact_points = scene.get_contact_points("cube1", "cube2");
    cout << "Contact points: ";
    for (int i = 0; i < contact_points.rows(); i++) {
        //cout << contact_points.row(i) << endl;
    }
    cout << endl;

    return 0;
}