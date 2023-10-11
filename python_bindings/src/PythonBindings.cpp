#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include "AssemblyCD.h"
namespace py = pybind11;

PYBIND11_MODULE(assembly_cd, m) {
    m.doc() = "Interface with PhysX to get the contact points between input shapes.";

    py::class_<Scene>(m, "Scene")
        .def(py::init<>(), "Initialize the PhysX scene.")
        .def("add_object", &Scene::add_object, "Add an object to the scene.",
                py::arg("id"), 
                py::arg("pose"), 
                py::arg("vertices"),
                py::arg("triangles"),
                py::arg("resolution") = 15,  
                py::arg("is_fixed") = false,
                py::arg("mass") = 1.0f, 
                py::arg("com") = (Vector3f() << 0.0f, 0.0f, 0.0f).finished(),
                py::arg("material_name") = "wood")
        .def("set_max_distance_factor", &Scene::set_max_distance_factor, "Sets a factor that multiplies that maximal distance an intersection point can be from the objects considered in contact.",
                py::arg("max_distance_factor"))
        .def("step_simulation", &Scene::step_simulation, "Step the simulation by a given time step.",
                py::arg("dt"))
        .def("get_contacted_objects", &Scene::get_contacted_objects, "Get the list of objects in contact with a given object.",
                py::arg("target_object"))
        .def("get_contact_points", &Scene::get_contact_points, "Get the contact points between two objects.",
                py::arg("id1"), 
                py::arg("id2"))
        .def("get_tri_vertices", &Scene::get_tri_vertices, "Get the triangle vertices of an object.",
                py::arg("id"))
        .def("get_tri_triangles", &Scene::get_tri_triangles, "Get the triangle indices of an object.",
                py::arg("id"))
        .def("get_tetra_vertices", &Scene::get_tetra_vertices, "Get the tetrahedron vertices of an object.",
                py::arg("id"))
        .def("get_tetra_indices", &Scene::get_tetra_indices, "Get the tetrahedron indices of an object.",
                py::arg("id"))
        .def("get_voxel_centres", &Scene::get_voxel_centres, "Get the occupied voxel centres of an object.",
                py::arg("id"))
        .def("get_voxel_side_lengths", &Scene::get_voxel_side_lengths, "Get the voxel side lengths of an object.",
                py::arg("id"))
        .def("merge_similar_contact_points", &Scene::merge_similar_contact_points, "Merge contact points that are close to each other.",
                py::arg("position_threshold") = 0, 
                py::arg("normal_threshold") = 0.1);

}
