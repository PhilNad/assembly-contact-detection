#include "AssemblyCD.h"
#include "test_cube.h"

void stress_test_object_creation(){
    //Stress-test where an object is moved around a lot, removed, re-created, and moved around again.
    int num_iterations = 0;
    int num_sub_iterations = 1;
    

    Scene scene;
    Cube cube = Cube(1, 1, 1);
    string id;
    Matrix4f pose;
    MatrixX3f contact_points;

    for(int i = 0; i < num_iterations; i++){
        //Create a new Scene
        scene = Scene();

        // Add another cube to the scene
        id = "cube4";
        pose << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0.5,
                0, 0, 0, 1;
        scene.add_object(id, pose, cube.vertices, cube.triangles, 30, true);
        
        // Add a cube to the scene
        id = "cube5";
        pose << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 1.4,
                0, 0, 0, 1;
        Cube cube3 = Cube(1, 1, 1);
        scene.add_object(id, pose, cube.vertices, cube.triangles, 30, true);

        scene.get_contacted_objects("cube5");

        for(int j = 0; j < num_sub_iterations; j++){

            //Move cube4
            pose << 1, 0, 0, 0,
                    0, 1, 0, (num_sub_iterations%2)/10,
                    0, 0, 1, 0.5,
                    0, 0, 0, 1;
            scene.set_object_pose("cube4", pose);

            // Add a cube to the scene
            id = "cube6";
            pose << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 2.4,
                    0, 0, 0, 1;
            scene.add_object(id, pose, cube.vertices, cube.triangles, 30, true);


            // Add a cube to the scene
            id = "cube7";
            pose << 1, 0, 0, 2,
                    0, 1, 0, 2,
                    0, 0, 1, 0.5,
                    0, 0, 0, 1;
            scene.add_object(id, pose, cube.vertices, cube.triangles, 30, true);

            scene.get_three_most_stable_contact_points("cube4");
            scene.get_three_most_stable_contact_points("cube5");
            scene.get_three_most_stable_contact_points("cube6");
            scene.get_three_most_stable_contact_points("cube7");

            scene.get_contacted_objects("cube5");
            contact_points = scene.get_all_penetrating_contact_points("cube5");
            cout << "Found " << contact_points.rows() << " penetrating points." << endl;

            scene.remove_object("cube6");
            scene.remove_object("cube7");
        }

        scene.remove_object("cube4");
        scene.remove_object("cube5");
    }
}