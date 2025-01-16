#include <iostream>
#include <vector>
#include <set>
#include "PxPhysicsAPI.h"
#include "eigen3/Eigen/Eigen"
#include "AssemblyCD.h"


#include "test_cube.h"

using namespace physx;
using namespace std;


int main(int argc, char** argv) {
    // Initialize the scene
    Scene scene;
    Matrix4f pose;

    //Create a fixed bottom cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube bot_cube = Cube(1, 1, 1);
    scene.add_object("bot_cube", pose, bot_cube.vertices, bot_cube.triangles, 15, true, true);

    //Create a mid cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 1.5,
            0, 0, 0, 1;
    Cube mid_cube = Cube(1, 1, 1);
    scene.add_object("mid_cube", pose, mid_cube.vertices, mid_cube.triangles, 15, true);

    //Create a top cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 2.5,
            0, 0, 0, 1;
    Cube top_cube = Cube(1, 1, 1);
    scene.add_object("top_cube", pose, top_cube.vertices, top_cube.triangles, 15, true);

    // Get contact forces between the objects
    vector<ContactForce> contact_forces = scene.get_contact_forces();
    Vector3f sum_forces = Vector3f::Zero();
    Vector3f sum_torques = Vector3f::Zero();
    for(auto& contact_force : contact_forces){
        cout << "Contact force between " << contact_force.object1_by_id << " and " << contact_force.object2_to_id << endl;
        cout << "\tPosition: " << contact_force.position.transpose() << endl;
        cout << "\tNormal: " << contact_force.normal_dir.transpose() << endl;
        cout << "\tLocal force: " << contact_force.local_force.transpose() << endl;
        cout << "\tFriction coefficient: " << contact_force.friction_coefficient << endl;
        VectorXf global_wrench = contact_force.get_global_wrench();
        Vector3f global_force = global_wrench.segment(0, 3);
        Vector3f global_torque = global_wrench.segment(3, 3);
        cout << "\tGlobal wrench: " << contact_force.get_global_wrench().transpose() << endl;
        sum_forces += global_force;
        sum_torques += global_torque;
    }

    //The sum of all forces should be close to [0, 0, 9.81]
    // and the sum of all torques should be close to [0, 0, 0].
    cout << "Sum of all forces: " << sum_forces.transpose() << endl;
    cout << "Sum of all torques: " << sum_torques.transpose() << endl;

    return 0;
}