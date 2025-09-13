# Google Test Integration

This directory contains both traditional test files and Google Test (gtest) based unit tests for the Assembly Contact Detection library.

## Structure

- **Traditional Tests**: `test.cpp` and `test_*.cpp` files contain the original test functions
- **Unit Tests**: `gtest_*.cpp` files contain structured unit tests using the Google Test framework

## Building and Running Tests

### Prerequisites

The gtest framework is automatically downloaded and built as part of the CMake configuration when `ACD_BUILD_GTEST_TESTS` is enabled (default: ON).

### Building

```bash
# Configure with gtest enabled (default)
cmake -S . -B build -DACD_BUILD_GTEST_TESTS=ON

# Build all targets including tests
cmake --build build --config Release --target all
```

### Running Tests

#### Traditional Tests
```bash
# Run the original test executable
./build/test/acd-tests
```

#### Unit Tests with gtest
```bash
# Run the gtest-based unit tests
./build/test/acd-unit-tests

# Run with verbose output
./build/test/acd-unit-tests --gtest_verbose

# Run specific test suites
./build/test/acd-unit-tests --gtest_filter="SceneTest*"
./build/test/acd-unit-tests --gtest_filter="ContactDetectionTest*"
./build/test/acd-unit-tests --gtest_filter="CubeTest*"

# List all available tests
./build/test/acd-unit-tests --gtest_list_tests
```

#### Using CTest
```bash
# Run tests through CTest
cd build
ctest

# Run with verbose output
ctest --verbose
```

## Test Organization

### gtest_object_creation.cpp
Tests for basic scene operations:
- Object creation and addition to scene
- Object removal
- Object pose setting
- Multiple object interactions
- Contact object queries

### gtest_contacts.cpp
Tests for contact detection functionality:
- Non-penetrating contact detection
- Penetrating contact detection
- Contact points between specific objects
- Contact force computation
- Edge cases (empty scene, single object)

### gtest_cube.cpp
Tests for the Cube utility class:
- Cube creation with different dimensions
- Geometric transformations (translation, scaling, rotation)
- Vertex and triangle validation
- Volume and dimension checks

## Adding New Tests

To add new gtest-based tests:

1. Create a new file with the prefix `gtest_` in the `test/src/` directory
2. Include the gtest headers and necessary library headers:
   ```cpp
   #include <gtest/gtest.h>
   #include "AssemblyCD.h"
   // other includes...
   ```
3. Use gtest macros for test organization:
   ```cpp
   // Test fixture for related tests
   class MyTestFixture : public ::testing::Test {
   protected:
       void SetUp() override { /* setup code */ }
       void TearDown() override { /* cleanup code */ }
   };
   
   // Individual test cases
   TEST_F(MyTestFixture, TestName) {
       EXPECT_EQ(expected, actual);
       ASSERT_TRUE(condition);
   }
   
   // Simple tests without fixtures
   TEST(TestSuiteName, TestName) {
       EXPECT_GT(value, 0);
   }
   ```

## Test Configuration

### CMake Options

- `ACD_BUILD_GTEST_TESTS`: Enable/disable gtest-based unit tests (default: ON)
- `ACD_DO_TESTS`: Enable/disable all tests including traditional ones (default: ON)

### Disabling gtest Tests

To build without gtest:
```bash
cmake -S . -B build -DACD_BUILD_GTEST_TESTS=OFF
```

This will only build the traditional test executable and skip gtest integration.

## Dependencies

The gtest integration automatically handles:
- Downloading Google Test framework (release-1.12.1)
- Building gtest as a static library
- Linking with pthread for multi-threading support
- Integration with the existing project dependencies (PhysX, Eigen, OSQP)

## Best Practices

1. **Use descriptive test names** that clearly indicate what is being tested
2. **Organize related tests** using test fixtures
3. **Include both positive and negative test cases**
4. **Test edge cases** (empty inputs, boundary conditions)
5. **Use appropriate assertions**:
   - `EXPECT_*` for non-fatal assertions
   - `ASSERT_*` for fatal assertions that should stop the test
6. **Keep tests focused** - each test should verify one specific behavior
7. **Make tests deterministic** - avoid random behavior or timing dependencies