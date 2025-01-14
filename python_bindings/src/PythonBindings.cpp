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
                py::arg("tri_vertices"),
                py::arg("tri_indices"),
                py::arg("resolution") = 15,
                py::arg("is_volumetric") = true,  
                py::arg("is_fixed") = false,
                py::arg("mass") = 1.0f, 
                py::arg("com") = (Vector3f() << 0.0f, 0.0f, 0.0f).finished(),
                py::arg("material_name") = "wood")
        .def("add_volumetric_object", &Scene::add_volumetric_object, "Add a volumetric object to the scene.",
                py::arg("id"), 
                py::arg("pose"), 
                py::arg("tri_vertices"),
                py::arg("tri_indices"),
                py::arg("tetra_vertices"),
                py::arg("tetra_indices"),
                py::arg("canary_sphere_positions"),
                py::arg("resolution") = 15,
                py::arg("is_fixed") = false,
                py::arg("mass") = 1.0f, 
                py::arg("com") = (Vector3f() << 0.0f, 0.0f, 0.0f).finished(),
                py::arg("material_name") = "wood")
        .def("get_all_object_ids", &Scene::get_all_object_ids, "Get the list of object ids in the scene.")
        .def("remove_object", &Scene::remove_object, "Remove an object from the scene.",
                py::arg("id"))
        .def("get_object_pose", &Scene::get_object_pose, "Get the pose of an object.",
                py::arg("id"))
        .def("set_object_pose", &Scene::set_object_pose, "Set the pose of an object.",
                py::arg("id"), 
                py::arg("pose"))
        .def("set_max_distance_factor", &Scene::set_max_distance_factor, "Sets a factor that multiplies that maximal distance an intersection point can be from the objects considered in contact.",
                py::arg("max_distance_factor"))
        .def("step_simulation", &Scene::step_simulation, "Step the simulation by a given time step.",
                py::arg("dt"))
        .def("get_contacted_objects", &Scene::get_contacted_objects, "Get the list of objects in contact with a given object.",
                py::arg("target_object"))
        .def("get_contact_point_positions", &Scene::get_contact_points_positions, "Get the positions of the contact points between two objects.",
                py::arg("id1"), 
                py::arg("id2"))
        .def("get_closest_contact_point", &Scene::get_closest_contact_point, "Get the closest contact point between two objects and a given point.",
                py::arg("id1"), 
                py::arg("id2"), 
                py::arg("point"))
        .def("get_penetrating_contact_point_positions", &Scene::get_penetrating_contact_point_positions, "Get the penetrating contact points between two objects.",
                py::arg("id1"), 
                py::arg("id2"))
        .def("get_all_contact_points", &Scene::get_all_contact_points, "Get the contact points between an object and all other objects.",
                py::arg("id"))
        .def("get_all_penetrating_contact_points", &Scene::get_all_penetrating_contact_points, "Get the penetrating contact points between an object and all other objects.",
                py::arg("id"))
        .def("get_contact_convex_hull", &Scene::get_contact_convex_hull, "Get the convex hull of the contact points on an object.",
                py::arg("id1"),
                py::arg("id2") = "",
                py::arg("vertex_limit") = 255)
        .def("get_three_most_stable_contact_points", &Scene::get_three_most_stable_contact_points, "Get the three most stable contact points on an object.",
                py::arg("id"),
                py::arg("hull_max_size") = 255,
                py::arg("random_third_point") = false)
        .def("get_tri_vertices", &Scene::get_tri_vertices, "Get the triangle vertices of an object.",
                py::arg("id"))
        .def("get_tri_triangles", &Scene::get_tri_triangles, "Get the triangle indices of an object.",
                py::arg("id"))
        .def("get_tetra_vertices", &Scene::get_tetra_vertices, "Get the tetrahedron vertices of an object.",
                py::arg("id"))
        .def("get_tetra_indices", &Scene::get_tetra_indices, "Get the tetrahedron indices of an object.",
                py::arg("id"))
        .def("get_canary_sphere_positions", &Scene::get_canary_sphere_positions, "Get the canary sphere positions of an object.",
                py::arg("id"))
        .def("get_voxel_centres", &Scene::get_voxel_centres, "Get the occupied voxel centres of an object.",
                py::arg("id"))
        .def("get_voxel_side_lengths", &Scene::get_voxel_side_lengths, "Get the voxel side lengths of an object.",
                py::arg("id"))
        .def("merge_similar_contact_points", &Scene::merge_similar_contact_points, "Merge contact points that are close to each other.",
                py::arg("contact_points"),
                py::arg("position_threshold") = 0, 
                py::arg("normal_threshold") = 0.1);

}
