#include "AssemblyCD.h"
#include "test_cube.h"

void test_object_creation(){
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
    scene.add_object(id, pose, cube1.vertices, cube1.triangles, 15, true);


    // Add another cube to the scene
    id = "cube2";
    pose << 1, 0, 0, 0,
            0, 1, 0, 0.25,
            0, 0, 1, 1.5, 
            0, 0, 0, 1;
    Cube cube2 = Cube(1, 1, 1);
    scene.add_object(id, pose, cube2.vertices, cube2.triangles, 15, true);

    // Add a cube to the scene
    id = "cube3";
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube cube3 = Cube(1, 1, 1);
    scene.add_object(id, pose, cube3.vertices, cube3.triangles, 15, true);
    
    //Remove cube1
    scene.remove_object("cube1");

    //Move cube2
    pose << 1, 0, 0, 0,
            0, 1, 0, 2.25,
            0, 0, 1, 1.5,// Use 1.5 for no penetration and 1.4 to trigger penetration
            0, 0, 0, 1;
    scene.set_object_pose("cube2", pose);

    //Remove cube3
    scene.remove_object("cube3");

    //New cube3 pose
    pose << 1, 0, 0, 0,
            0, 1, 0, 2,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    //Add cube3 in new pose
    scene.add_object("cube3", pose, cube3.vertices, cube3.triangles, 15, true);

    // Get the list of objects in contact with the cube
    unordered_set<string> contacted_objects = scene.get_contacted_objects("cube3");
    cout << "Contacted objects: ";
    for(auto it = contacted_objects.begin(); it != contacted_objects.end(); ++it){
        cout << *it << " ";
    }
    cout << endl;
}