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
                py::arg("is_fixed") = false,
                py::arg("mass") = 1.0f, 
                py::arg("com") = (Vector3f() << 0.0f, 0.0f, 0.0f).finished(),
                py::arg("material_name") = "wood")
        .def("step_simulation", &Scene::step_simulation, "Step the simulation by a given time step.",
                py::arg("dt"))
        .def("get_contacted_objects", &Scene::get_contacted_objects, "Get the list of objects in contact with a given object.",
                py::arg("target_object"))
        .def("get_contact_points", &Scene::get_contact_points, "Get the contact points between two objects.",
                py::arg("id1"), 
                py::arg("id2"));

}
