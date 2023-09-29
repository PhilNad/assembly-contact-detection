# Assembly Contact Detection with PhysX
This library is a wrapper around the PhysX library to provide a simple interface for contact detection between rigid bodies. For now, only contact detection is supported, but in the future, this library could be extended to produce static scenes through dynamics simulation.

This library grew from the need for a fast collision detection algorithm that could work with non-convex objects. Building atop NVIDIA'a PhysX library enables this library to be fast and accurate, even when running on the CPU (although GPU support is obviously possible). This comes at the cost of several headaches when trying to use the library, as [the documentation](https://nvidia-omniverse.github.io/PhysX/physx/5.3.0/docs/MigrationTo53.html) is incomplete.

An additional requirement is that the detected contact points covers as much as possible of the interface between colliding objects. This is typically not supported by most collision detection libraries as the dynamical behaviour of objects in collision can be more efficiently simulated using a small number of contact points at which impacts occur. However, for assembly planning, it is important to think about potential counteracting forces that could appear at any point of contact between rigid objects. To facilitate the detection of a great number of contact points, a voxelization of the surface of the objects is performed. By detecting collisions between pairs of voxels from different objects, a large number of contact points can be produced. This technique has the added benefit of having no requirements on the triangular mesh provided as input (i.e. any triangle soup can be used), making it easy to interface with Trimesh and Open3D through the Python bindings.

## Dependencies
- [PhysX](https://github.com/NVIDIA-Omniverse/PhysX) : Download the latest release, and unzip it in a convenient (permanent) location.
- [Eigen](https://eigen.tuxfamily.org/)
- [PyBind11](https://pybind11.readthedocs.io/en/stable/index.html) (this is included in the repository as an external module).
- `sudo apt install libglew-dev freeglut3-dev clang` to be able to compile PhysX's example snippets
- A compiler for C++ 11

## Prerequisites
1. Create a symlink to the PhysX directory in this project's directory:
```bash
ln -s /home/phil/PhysX/physx physx
```
2. Navigate to the PhysX directory and run
```bash
./generate_projects.sh linux
```
3. Navigate to the `physx/compiler/linux-release` directory and run 
```bash
make PhysXPvdSDk PhysXExtensions PhysXCooking
``` 
that will trigger the compilation of the PhysX libraries. Note that when developing the library, you should be using the `checked` version of the libraries, which can be compiled by navigating to the `physx/compiler/linux-checked` directory instead.

4. Navigate to this project's directory and run these commands to install PyBind11 on the computer:
```bash
> git submodule update --init --recursive
> cd extern/pybind11
> cmake -S . -B build
> cmake --build build
> sudo cmake --install build
```
5. Build the library with: 
```bash
> cmake --build "assembly-contact-detection/build" --config Release --target clean
> cmake --build "assembly-contact-detection/build" --config Release --target all
```
6. Install everything with:
```bash
> sudo cmake --install build
```

## Examples
The library can be used in C++ or through Python bindings. Test examples are provided in the `test` directory.
- `test/src/test.cpp` : C++ example
- `test/test.py` : Python example

### Minimal Example
```python
import open3d as o3d
import numpy as np
import assembly_cd as acd


#A cone is atop a cube
cube = o3d.geometry.TriangleMesh.create_box(width=1.0, height=1.0, depth=1.0)
cone = o3d.geometry.TriangleMesh.create_cone(radius=0.5, height=1.0)
cone.translate((0.5, 0.5, 1))

#Instanciate the scene and add objects
scene = acd.Scene()
scene.add_object("cube", np.identity(4), np.asarray(cube.vertices), np.asarray(cube.triangles))
scene.add_object("cone", np.identity(4), np.asarray(cone.vertices), np.asarray(cone.triangles))

#Detect contact points between the two objects
contact_points = scene.get_contact_points("cube", "cone")

#Draw the scene and the contact points
# Tip: In the GUI menu, select Actions and click on 'Show Settings'. 
# Then, in the settings panel, select 'Wireframe' to see through objects,
# and set the background color to white.
contact_point_cloud = o3d.geometry.PointCloud()
contact_point_cloud.points = o3d.utility.Vector3dVector(contact_points)
o3d.visualization.draw([cube, cone, contact_point_cloud])
```