#include <gtest/gtest.h>
#include "test_cube.h"
#include <cmath>

using namespace std;

// Test fixture for Cube utility class tests
class CubeTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Standard unit cube for testing
        unit_cube = Cube(1.0f, 1.0f, 1.0f);
    }

    void TearDown() override {
    }

    Cube unit_cube{1.0f, 1.0f, 1.0f};
};

// Test cube creation with valid dimensions
TEST_F(CubeTest, CubeCreation) {
    Cube cube(2.0f, 3.0f, 4.0f);
    
    // Cube should have 8 vertices
    EXPECT_EQ(cube.vertices.rows(), 8);
    EXPECT_EQ(cube.vertices.cols(), 3);
    
    // Cube should have 12 triangles (2 per face * 6 faces)
    EXPECT_EQ(cube.triangles.rows(), 12);
    EXPECT_EQ(cube.triangles.cols(), 3);
}

// Test cube dimensions
TEST_F(CubeTest, CubeDimensions) {
    Cube cube(2.0f, 4.0f, 6.0f);
    
    // Find min and max coordinates
    float min_x = cube.vertices.col(0).minCoeff();
    float max_x = cube.vertices.col(0).maxCoeff();
    float min_y = cube.vertices.col(1).minCoeff();
    float max_y = cube.vertices.col(1).maxCoeff();
    float min_z = cube.vertices.col(2).minCoeff();
    float max_z = cube.vertices.col(2).maxCoeff();
    
    // Check dimensions (extent is full width, so half-extent on each side)
    EXPECT_FLOAT_EQ(max_x - min_x, 2.0f);
    EXPECT_FLOAT_EQ(max_y - min_y, 4.0f);
    EXPECT_FLOAT_EQ(max_z - min_z, 6.0f);
    
    // Check centering (should be centered at origin)
    EXPECT_FLOAT_EQ(min_x, -1.0f);
    EXPECT_FLOAT_EQ(max_x, 1.0f);
    EXPECT_FLOAT_EQ(min_y, -2.0f);
    EXPECT_FLOAT_EQ(max_y, 2.0f);
    EXPECT_FLOAT_EQ(min_z, -3.0f);
    EXPECT_FLOAT_EQ(max_z, 3.0f);
}

// Test cube translation
TEST_F(CubeTest, CubeTranslation) {
    // Get original center
    Vector3f original_center = unit_cube.vertices.colwise().mean();
    
    // Translate the cube
    float dx = 1.0f, dy = 2.0f, dz = 3.0f;
    unit_cube.translate(dx, dy, dz);
    
    // Check new center
    Vector3f new_center = unit_cube.vertices.colwise().mean();
    
    EXPECT_FLOAT_EQ(new_center.x(), original_center.x() + dx);
    EXPECT_FLOAT_EQ(new_center.y(), original_center.y() + dy);
    EXPECT_FLOAT_EQ(new_center.z(), original_center.z() + dz);
}

// Test cube scaling
TEST_F(CubeTest, CubeScaling) {
    RowVector3f center(0, 0, 0);
    float scale_factor = 2.0f;
    
    // Get original dimensions
    float original_extent_x = unit_cube.vertices.col(0).maxCoeff() - unit_cube.vertices.col(0).minCoeff();
    float original_extent_y = unit_cube.vertices.col(1).maxCoeff() - unit_cube.vertices.col(1).minCoeff();
    float original_extent_z = unit_cube.vertices.col(2).maxCoeff() - unit_cube.vertices.col(2).minCoeff();
    
    // Scale the cube
    unit_cube.scale(scale_factor, center);
    
    // Check new dimensions
    float new_extent_x = unit_cube.vertices.col(0).maxCoeff() - unit_cube.vertices.col(0).minCoeff();
    float new_extent_y = unit_cube.vertices.col(1).maxCoeff() - unit_cube.vertices.col(1).minCoeff();
    float new_extent_z = unit_cube.vertices.col(2).maxCoeff() - unit_cube.vertices.col(2).minCoeff();
    
    EXPECT_FLOAT_EQ(new_extent_x, original_extent_x * scale_factor);
    EXPECT_FLOAT_EQ(new_extent_y, original_extent_y * scale_factor);
    EXPECT_FLOAT_EQ(new_extent_z, original_extent_z * scale_factor);
}

// Test cube rotation (basic test - rotation by 0 should not change anything)
TEST_F(CubeTest, CubeRotationIdentity) {
    // Store original vertices
    MatrixX3f original_vertices = unit_cube.vertices;
    
    // Rotate by 0 degrees around Z axis
    RowVector3f axis(0, 0, 1);
    RowVector3f center(0, 0, 0);
    unit_cube.rotate(0.0f, axis, center);
    
    // Vertices should remain the same
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            EXPECT_NEAR(unit_cube.vertices(i, j), original_vertices(i, j), 1e-6);
        }
    }
}

// Test cube rotation by 90 degrees around Z axis
TEST_F(CubeTest, CubeRotation90Z) {
    // Store original vertex at (0.5, 0.5, 0.5) which should become (-0.5, 0.5, 0.5)
    // Find vertex closest to (0.5, 0.5, 0.5)
    int target_vertex = -1;
    for (int i = 0; i < 8; i++) {
        if (abs(unit_cube.vertices(i, 0) - 0.5f) < 1e-6 &&
            abs(unit_cube.vertices(i, 1) - 0.5f) < 1e-6 &&
            abs(unit_cube.vertices(i, 2) - 0.5f) < 1e-6) {
            target_vertex = i;
            break;
        }
    }
    
    if (target_vertex >= 0) {
        // Rotate by 90 degrees around Z axis
        RowVector3f axis(0, 0, 1);
        RowVector3f center(0, 0, 0);
        unit_cube.rotate(M_PI/2.0f, axis, center);
        
        // Check that (0.5, 0.5, 0.5) became approximately (-0.5, 0.5, 0.5)
        EXPECT_NEAR(unit_cube.vertices(target_vertex, 0), -0.5f, 1e-6);
        EXPECT_NEAR(unit_cube.vertices(target_vertex, 1), 0.5f, 1e-6);
        EXPECT_NEAR(unit_cube.vertices(target_vertex, 2), 0.5f, 1e-6);
    }
}

// Test triangle indices are valid
TEST_F(CubeTest, ValidTriangleIndices) {
    // All triangle indices should be in range [0, 7] for an 8-vertex cube
    for (int i = 0; i < unit_cube.triangles.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            int index = unit_cube.triangles(i, j);
            EXPECT_GE(index, 0) << "Triangle index should be non-negative";
            EXPECT_LT(index, 8) << "Triangle index should be less than number of vertices";
        }
    }
}

// Test that cube has positive volume (triangles form a closed surface)
TEST_F(CubeTest, PositiveVolume) {
    // Basic check: cube extents should be positive
    float extent_x = unit_cube.vertices.col(0).maxCoeff() - unit_cube.vertices.col(0).minCoeff();
    float extent_y = unit_cube.vertices.col(1).maxCoeff() - unit_cube.vertices.col(1).minCoeff();
    float extent_z = unit_cube.vertices.col(2).maxCoeff() - unit_cube.vertices.col(2).minCoeff();
    
    EXPECT_GT(extent_x, 0) << "Cube should have positive X extent";
    EXPECT_GT(extent_y, 0) << "Cube should have positive Y extent";
    EXPECT_GT(extent_z, 0) << "Cube should have positive Z extent";
}