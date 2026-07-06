#include "basements/editor/scripting/python_bridge.h"
#include <pybind11/embed.h>
#include <iostream>

#include "basements/engine/scene_graph.h"
#include "basements/core/math/vec3.h"

namespace py = pybind11;

namespace basements {
namespace editor {

// Global interpreter scope
// unique_ptr cannot be used easily with static duration scoped_interpreter check
// We will manage it carefully. 
// Actually pybind11::scoped_interpreter should be instantiated once in main or initialize.
static py::scoped_interpreter* g_python_interpreter = nullptr;

void PythonBridge::initialize() {
    if (!g_python_interpreter) {
        try {
            g_python_interpreter = new py::scoped_interpreter();
            std::cout << "🐍 Python initialized successfully." << std::endl;
            
            // Import main module to check health
            py::module_::import("sys");
            
            bind_api();
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize Python: " << e.what() << std::endl;
        }
    }
}

void PythonBridge::shutdown() {
    if (g_python_interpreter) {
        delete g_python_interpreter;
        g_python_interpreter = nullptr;
    }
}

void PythonBridge::execute_string(const std::string& code) {
    try {
        py::exec(code);
    } catch (py::error_already_set& e) {
        std::cerr << "Python Error: " << e.what() << std::endl;
    }
}

void PythonBridge::bind_api() {
    // Bind 'basements' module
    // Note: In embedded mode, PYBIND11_EMBEDDED_MODULE is usually used at global scope.
    // However, we can also manually add to sys.modules or use raw C API if needed,
    // but pybind11 provides a macro for restartable modules.
    
    // Since we are embedding, we can't easily use PYBIND11_MODULE(). 
    // We use PYBIND11_EMBEDDED_MODULE at global scope (see below).
}

} // namespace editor
} // namespace basements

// Access to Editor App (External Linkage)
#include "basements/editor/editor_app.h"

// Embedded module definition
// Embedded module definition
PYBIND11_EMBEDDED_MODULE(basements, m) {
    using namespace basements;
    using namespace basements::editor;
    using namespace basements::math;

    m.doc() = "Basements Engine Python API - Optimized for C++ Performance";
    
    // Bind global editor access
    m.def("get_scene", []() -> basements::editor::SceneGraph& {
        auto* app = basements::editor::get_editor_app();
        if (app) return app->get_scene();
        throw std::runtime_error("Editor App not found");
    }, pybind11::return_value_policy::reference);

    // ============================================================
    // Math Types (Direct Memory mapping where possible)
    // ============================================================
    py::class_<Vec3>(m, "Vec3")
        .def(py::init<float, float, float>(), py::arg("x")=0.0f, py::arg("y")=0.0f, py::arg("z")=0.0f)
        .def_readwrite("x", &Vec3::x)
        .def_readwrite("y", &Vec3::y)
        .def_readwrite("z", &Vec3::z)
        .def("length", &Vec3::length)
        .def("__add__", [](const Vec3& a, const Vec3& b) { return a + b; })
        .def("__repr__", [](const Vec3& v) {
            return "Vec3(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
        });

    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<float, float, float, float>(), py::arg("w")=1.0f, py::arg("x")=0.0f, py::arg("y")=0.0f, py::arg("z")=0.0f)
        .def_static("identity", &Quaternion::identity)
        .def_static("from_euler", &Quaternion::from_euler)
        .def("to_euler", [](const Quaternion& q) {
            float p, y, r;
            q.to_euler(p, y, r);
            return py::make_tuple(p, y, r); // Returns tuple (pitch, yaw, roll)
        })
        .def("__repr__", [](const Quaternion& q) {
            return "Quaternion(" + std::to_string(q.w) + ", " + std::to_string(q.x) + ", " + std::to_string(q.y) + ", " + std::to_string(q.z) + ")";
        });

    // ============================================================
    // Core Types
    // ============================================================
    py::enum_<NodeType>(m, "NodeType")
        .value("ROOT", NodeType::ROOT)
        .value("RIGID_BODY", NodeType::RIGID_BODY)
        .value("JOINT", NodeType::JOINT)
        .value("GROUP", NodeType::GROUP)
        .export_values();

    py::enum_<ShapeType>(m, "ShapeType")
        .value("BOX", ShapeType::BOX)
        .value("SPHERE", ShapeType::SPHERE)
        .value("CAPSULE", ShapeType::CAPSULE)
        .value("MESH", ShapeType::MESH)
        .export_values();

    // Bind SceneNode
    py::class_<SceneNode, std::unique_ptr<SceneNode, py::nodelete>>(m, "SceneNode") 
        .def_readonly("id", &SceneNode::id)
        .def_readwrite("name", &SceneNode::name)
        .def_readwrite("position", &SceneNode::local_position)
        .def_readwrite("rotation", &SceneNode::local_rotation) 
        .def_readwrite("scale", &SceneNode::local_scale)
        .def_readwrite("parent_id", &SceneNode::parent_id)
        .def("set_mass", [](SceneNode& self, float mass) { self.physics.mass = mass; })
        .def("get_mass", [](SceneNode& self) { return self.physics.mass; });

    // Bind SceneGraph
    py::class_<SceneGraph>(m, "SceneGraph")
        .def("get_node", &SceneGraph::get_node, py::return_value_policy::reference) 
        .def("create_node", &SceneGraph::create_node)
        .def("create_rigid_body", &SceneGraph::create_rigid_body)
        .def("remove_node", &SceneGraph::remove_node)
        .def("get_root_nodes", &SceneGraph::get_root_children);

    // Bind Log
    m.def("log", [](const std::string& msg) {
        std::cout << "[Python] " << msg << std::endl;
    });
}
