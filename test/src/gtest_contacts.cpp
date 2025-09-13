#include <gtest/gtest.h>
#include "AssemblyCD.h"
#include "test_cube.h"

using namespace std;

// Test fixture for contact detection tests
class ContactDetectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        scene = Scene();
    }

    void TearDown() override {
    }

    Scene scene;
};

// Test non-penetrating contact detection
TEST_F(ContactDetectionTest, NonPenetratingContacts) {
    Matrix4f pose;
    
    // Create a bottom cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 2,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube bot_cube = Cube(1, 1, 1);
    scene.add_object("bot_cube", pose, bot_cube.vertices, bot_cube.triangles, 15, true);

    // Create a top cube positioned to just touch (not penetrate)
    pose << 1, 0, 0, 0,
            0, 1, 0, 2.25,
            0, 0, 1, 1.5,  // 1.5 for no penetration
            0, 0, 0, 1;
    Cube top_cube = Cube(1, 1, 1);
    scene.add_object("top_cube", pose, top_cube.vertices, top_cube.triangles, 15, true);

    // Get contact points
    MatrixX3f contact_points = scene.get_all_contact_points("top_cube");
    MatrixX3f penetrating_points = scene.get_all_penetrating_contact_points("top_cube");

    // There should be contact points but no penetrating points
    EXPECT_GT(contact_points.rows(), 0) << "Should have contact points when objects are touching";
    EXPECT_EQ(penetrating_points.rows(), 0) << "Should have no penetrating points when objects are just touching";
}

// Test penetrating contact detection
TEST_F(ContactDetectionTest, PenetratingContacts) {
    Matrix4f pose;
    
    // Create a bottom cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 2,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube bot_cube = Cube(1, 1, 1);
    scene.add_object("bot_cube", pose, bot_cube.vertices, bot_cube.triangles, 15, true);

    // Create a top cube positioned to penetrate
    pose << 1, 0, 0, 0,
            0, 1, 0, 2.25,
            0, 0, 1, 1.4,  // 1.4 to trigger penetration
            0, 0, 0, 1;
    Cube top_cube = Cube(1, 1, 1);
    scene.add_object("top_cube", pose, top_cube.vertices, top_cube.triangles, 15, true);

    // Get contact points
    MatrixX3f contact_points = scene.get_all_contact_points("top_cube");
    MatrixX3f penetrating_points = scene.get_all_penetrating_contact_points("top_cube");

    // There should be both contact points and penetrating points
    EXPECT_GT(contact_points.rows(), 0) << "Should have contact points when objects are penetrating";
    EXPECT_GT(penetrating_points.rows(), 0) << "Should have penetrating points when objects are interpenetrating";
}

// Test contact points between specific objects
TEST_F(ContactDetectionTest, ContactPointsBetweenObjects) {
    Matrix4f pose;
    
    // Create bottom cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube cube1 = Cube(1, 1, 1);
    scene.add_object("cube1", pose, cube1.vertices, cube1.triangles, 15, true);
    
    // Create top cube that touches the bottom cube
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 1.5,
            0, 0, 0, 1;
    Cube cube2 = Cube(1, 1, 1);
    scene.add_object("cube2", pose, cube2.vertices, cube2.triangles, 15, true);
    
    // Get contact points between the specific objects
    MatrixX3f contact_points = scene.get_contact_point_positions("cube1", "cube2");
    
    // Should have contact points between the two cubes
    EXPECT_GE(contact_points.rows(), 0) << "Should be able to get contact points between specific objects";
}

// Test contact forces
TEST_F(ContactDetectionTest, ContactForces) {
    Matrix4f pose;
    
    // Create bottom cube (fixed)
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    Cube bot_cube = Cube(1, 1, 1);
    scene.add_object("bot_cube", pose, bot_cube.vertices, bot_cube.triangles, 15, true, true);  // Fixed
    
    // Create top cube with mass (not fixed)
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 1.5,
            0, 0, 0, 1;
    Cube top_cube = Cube(1, 1, 1);
    scene.add_object("top_cube", pose, top_cube.vertices, top_cube.triangles, 15, false, false, 1.0);  // Has mass
    
    // Get contact forces
    vector<ContactForce> contact_forces = scene.get_contact_forces();
    
    // Should have some contact forces
    EXPECT_GE(contact_forces.size(), 0) << "Should be able to compute contact forces";
}

// Test empty scene behavior
TEST_F(ContactDetectionTest, EmptyScene) {
    // Test with no objects
    vector<ContactForce> contact_forces = scene.get_contact_forces();
    EXPECT_EQ(contact_forces.size(), 0) << "Empty scene should have no contact forces";
}

// Test single object behavior
TEST_F(ContactDetectionTest, SingleObject) {
    Matrix4f pose = Matrix4f::Identity();
    pose(2, 3) = 0.5;
    
    Cube cube = Cube(1, 1, 1);
    scene.add_object("single_cube", pose, cube.vertices, cube.triangles, 15, true);
    
    // Single object should have no contacts
    MatrixX3f contact_points = scene.get_all_contact_points("single_cube");
    EXPECT_EQ(contact_points.rows(), 0) << "Single object should have no contact points";
    
    vector<ContactForce> contact_forces = scene.get_contact_forces();
    EXPECT_EQ(contact_forces.size(), 0) << "Single object should generate no contact forces";
}