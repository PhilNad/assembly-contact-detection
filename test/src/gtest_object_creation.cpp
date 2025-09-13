#include <gtest/gtest.h>
#include "AssemblyCD.h"
#include "test_cube.h"
#include <unordered_set>

using namespace std;

// Test fixture for scene-based tests
class SceneTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize scene for each test
        scene = Scene();
    }

    void TearDown() override {
        // Clean up after each test
    }

    Scene scene;
};

// Test basic object creation and addition to scene
TEST_F(SceneTest, ObjectCreation) {
    // Create a cube and add it to the scene
    string id = "test_cube";
    Matrix4f pose;
    pose << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0.5,
            0, 0, 0, 1;
    
    Cube cube = Cube(1, 1, 1);
    
    // This should not throw an exception
    EXPECT_NO_THROW(scene.add_object(id, pose, cube.vertices, cube.triangles, 15, true));
}

// Test object removal
TEST_F(SceneTest, ObjectRemoval) {
    // Add a cube first
    string id = "test_cube";
    Matrix4f pose = Matrix4f::Identity();
    pose(2, 3) = 0.5;  // Set z position
    
    Cube cube = Cube(1, 1, 1);
    scene.add_object(id, pose, cube.vertices, cube.triangles, 15, true);
    
    // Remove the object - should not throw
    EXPECT_NO_THROW(scene.remove_object(id));
}

// Test object pose setting
TEST_F(SceneTest, ObjectPoseSetting) {
    // Add a cube first
    string id = "test_cube";
    Matrix4f initial_pose = Matrix4f::Identity();
    initial_pose(2, 3) = 0.5;
    
    Cube cube = Cube(1, 1, 1);
    scene.add_object(id, initial_pose, cube.vertices, cube.triangles, 15, true);
    
    // Change the pose
    Matrix4f new_pose = Matrix4f::Identity();
    new_pose(2, 3) = 1.5;  // Move up
    
    EXPECT_NO_THROW(scene.set_object_pose(id, new_pose));
}

// Test multiple object creation and interaction
TEST_F(SceneTest, MultipleObjectCreation) {
    Matrix4f pose1, pose2, pose3;
    
    // Add first cube
    pose1 << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0.5,
             0, 0, 0, 1;
    Cube cube1 = Cube(1, 1, 1);
    EXPECT_NO_THROW(scene.add_object("cube1", pose1, cube1.vertices, cube1.triangles, 15, true));
    
    // Add second cube
    pose2 << 1, 0, 0, 0,
             0, 1, 0, 0.25,
             0, 0, 1, 1.5,
             0, 0, 0, 1;
    Cube cube2 = Cube(1, 1, 1);
    EXPECT_NO_THROW(scene.add_object("cube2", pose2, cube2.vertices, cube2.triangles, 15, true));
    
    // Add third cube
    pose3 << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0.5,
             0, 0, 0, 1;
    Cube cube3 = Cube(1, 1, 1);
    EXPECT_NO_THROW(scene.add_object("cube3", pose3, cube3.vertices, cube3.triangles, 15, true));
    
    // Remove first cube
    EXPECT_NO_THROW(scene.remove_object("cube1"));
    
    // Move second cube
    Matrix4f new_pose2;
    new_pose2 << 1, 0, 0, 0,
                 0, 1, 0, 2.25,
                 0, 0, 1, 1.5,
                 0, 0, 0, 1;
    EXPECT_NO_THROW(scene.set_object_pose("cube2", new_pose2));
}

// Test getting contacted objects
TEST_F(SceneTest, ContactedObjects) {
    // Create two cubes that should be in contact
    Matrix4f pose1, pose2;
    
    // Bottom cube
    pose1 << 1, 0, 0, 0,
             0, 1, 0, 2,
             0, 0, 1, 0.5,
             0, 0, 0, 1;
    Cube cube1 = Cube(1, 1, 1);
    scene.add_object("bottom_cube", pose1, cube1.vertices, cube1.triangles, 15, true);
    
    // Top cube positioned to touch the bottom cube
    pose2 << 1, 0, 0, 0,
             0, 1, 0, 2,
             0, 0, 1, 1.5,  // Positioned to just touch
             0, 0, 0, 1;
    Cube cube2 = Cube(1, 1, 1);
    scene.add_object("top_cube", pose2, cube2.vertices, cube2.triangles, 15, true);
    
    // Get contacted objects for the top cube
    unordered_set<string> contacted_objects = scene.get_contacted_objects("top_cube");
    
    // Should have at least one contacted object
    EXPECT_GT(contacted_objects.size(), 0);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}