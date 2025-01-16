#include "AssemblyCD.h"
#include "test_cube.h"

void test_nonpenetrating_contacts(){
    // Initialize the scene
    Scene scene;
    Matrix4f pose;

    //Create a bottom cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 2,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube bot_cube = Cube(1, 1, 1);
    scene.add_object("bot_cube", pose, bot_cube.vertices, bot_cube.triangles, 15, true);

    //Create a top cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 2.25,
            0, 0, 1, 1.5,// Use 1.5 for no penetration and 1.4 to trigger penetration
            0, 0, 0, 1;
    Cube top_cube = Cube(1, 1, 1);
    scene.add_object("top_cube", pose, top_cube.vertices, top_cube.triangles, 15, true);

    // Get the contact points between the two objects
    MatrixX3f contact_points = scene.get_all_contact_points("top_cube");
    cout << "Found " << contact_points.rows() << " contact points." << endl;
    contact_points = scene.get_all_penetrating_contact_points("top_cube");
    cout << "Found " << contact_points.rows() << " penetrating points." << endl;

    //Result: There should be many contact points and zero penetrating points.
}

void test_penetrating_contacts()
{
    // Initialize the scene
    Scene scene;
    Matrix4f pose;

    //Create a bottom cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 2,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube bot_cube = Cube(1, 1, 1);
    scene.add_object("bot_cube", pose, bot_cube.vertices, bot_cube.triangles, 15, true);

    //Create a top cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 2.25,
            0, 0, 1, 1.4,
            0, 0, 0, 1;
    Cube top_cube = Cube(1, 1, 1);
    scene.add_object("top_cube", pose, top_cube.vertices, top_cube.triangles, 15, true);

    // Get the contact points between the two objects
    MatrixX3f contact_points = scene.get_all_contact_points("top_cube");
    cout << "Found " << contact_points.rows() << " contact points." << endl;
    contact_points = scene.get_all_penetrating_contact_points("top_cube");
    cout << "Found " << contact_points.rows() << " penetrating points." << endl;

    //Result: There should be many contact points and many penetrating points.
}

//TODO: Test convex hull computation
/*
// Print the position of the five farthest contact points
    MatrixX3f hull_contacts = scene.get_contact_convex_hull("top_cube", "", 5);
    cout << "Position of the five farthest contact points:" << endl;
    for(int i = 0; i < hull_contacts.rows(); i++){
        cout << hull_contacts.row(i) << endl;
    }
*/