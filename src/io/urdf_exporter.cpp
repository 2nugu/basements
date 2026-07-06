#include "basements/io/urdf_parser.h"
#include <tinyxml2.h>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <string>
#include <cmath>

using namespace tinyxml2;

namespace basements {
namespace io {

// ========== Export to String/File ==========

std::string URDFParser::export_to_string(const URDFRobot& robot) {
    XMLDocument doc;
    
    // XML declaration
    XMLDeclaration* decl = doc.NewDeclaration();
    doc.InsertFirstChild(decl);
    
    // Root <robot> element
    XMLElement* robot_elem = doc.NewElement("robot");
    robot_elem->SetAttribute("name", robot.name.c_str());
    doc.InsertEndChild(robot_elem);
    
    // Write global materials
    for (const auto& material : robot.materials) {
        write_material(robot_elem, material);
    }
    
    // Write all links
    for (const auto& link : robot.links) {
        write_link(robot_elem, link);
    }
    
    // Write all joints
    for (const auto& joint : robot.joints) {
        write_joint(robot_elem, joint);
    }
    
    // Convert to string
    XMLPrinter printer;
    doc.Print(&printer);
    return std::string(printer.CStr());
}

bool URDFParser::export_to_file(const URDFRobot& robot, const std::string& filepath) {
    std::string xml = export_to_string(robot);
    
    std::ofstream file(filepath);
    if (!file.is_open()) {
        last_error_ = "Failed to open file for writing: " + filepath;
        return false;
    }
    
    file << xml;
    file.close();
    return true;
}

// ========== Helper: Write Pose ==========

void URDFParser::write_pose(XMLElement* parent, const URDFPose& pose, const char* tag_name) {
    XMLDocument* doc = parent->GetDocument();
    XMLElement* elem = doc->NewElement(tag_name);
    
    // Write xyz
    std::ostringstream xyz;
    xyz << pose.position.x << " " << pose.position.y << " " << pose.position.z;
    elem->SetAttribute("xyz", xyz.str().c_str());
    
    // Convert quaternion to RPY
    // This is approximate - for exact conversion use proper quaternion math
    float roll = std::atan2(2.0f * (pose.orientation.w * pose.orientation.x + 
                                     pose.orientation.y * pose.orientation.z),
                            1.0f - 2.0f * (pose.orientation.x * pose.orientation.x + 
                                           pose.orientation.y * pose.orientation.y));
    float pitch = std::asin(2.0f * (pose.orientation.w * pose.orientation.y - 
                                     pose.orientation.z * pose.orientation.x));
    float yaw = std::atan2(2.0f * (pose.orientation.w * pose.orientation.z + 
                                    pose.orientation.x * pose.orientation.y),
                           1.0f - 2.0f * (pose.orientation.y * pose.orientation.y + 
                                          pose.orientation.z * pose.orientation.z));
    
    std::ostringstream rpy;
    rpy << roll << " " << pitch << " " << yaw;
    elem->SetAttribute("rpy", rpy.str().c_str());
    
    parent->InsertEndChild(elem);
}

// ========== Helper: Write Geometry ==========

void URDFParser::write_geometry(XMLElement* parent, const URDFGeometry& geometry) {
    XMLDocument* doc = parent->GetDocument();
    XMLElement* geom_elem = doc->NewElement("geometry");
    
    switch (geometry.type) {
        case URDFGeometry::BOX: {
            XMLElement* box = doc->NewElement("box");
            std::ostringstream size;
            size << geometry.size.x << " " << geometry.size.y << " " << geometry.size.z;
            box->SetAttribute("size", size.str().c_str());
            geom_elem->InsertEndChild(box);
            break;
        }
        case URDFGeometry::SPHERE: {
            XMLElement* sphere = doc->NewElement("sphere");
            sphere->SetAttribute("radius", geometry.radius);
            geom_elem->InsertEndChild(sphere);
            break;
        }
        case URDFGeometry::CYLINDER: {
            XMLElement* cylinder = doc->NewElement("cylinder");
            cylinder->SetAttribute("radius", geometry.radius);
            cylinder->SetAttribute("length", geometry.length);
            geom_elem->InsertEndChild(cylinder);
            break;
        }
        case URDFGeometry::MESH: {
            XMLElement* mesh = doc->NewElement("mesh");
            mesh->SetAttribute("filename", geometry.mesh_filename.c_str());
            std::ostringstream scale;
            scale << geometry.mesh_scale.x << " " << geometry.mesh_scale.y << " " << geometry.mesh_scale.z;
            mesh->SetAttribute("scale", scale.str().c_str());
            geom_elem->InsertEndChild(mesh);
            break;
        }
    }
    
    parent->InsertEndChild(geom_elem);
}

// ========== Helper: Write Material ==========

void URDFParser::write_material(XMLElement* parent, const URDFMaterial& material) {
    XMLDocument* doc = parent->GetDocument();
    XMLElement* mat_elem = doc->NewElement("material");
    mat_elem->SetAttribute("name", material.name.c_str());
    
    // Write color
    XMLElement* color = doc->NewElement("color");
    std::ostringstream rgba;
    rgba << material.color.r << " " << material.color.g << " " 
         << material.color.b << " " << material.color.a;
    color->SetAttribute("rgba", rgba.str().c_str());
    mat_elem->InsertEndChild(color);
    
    // Write texture if present
    if (!material.texture_filename.empty()) {
        XMLElement* texture = doc->NewElement("texture");
        texture->SetAttribute("filename", material.texture_filename.c_str());
        mat_elem->InsertEndChild(texture);
    }
    
    parent->InsertEndChild(mat_elem);
}

// ========== Helper: Write Inertial ==========

void URDFParser::write_inertial(XMLElement* parent, const URDFInertial& inertial) {
    XMLDocument* doc = parent->GetDocument();
    XMLElement* inertial_elem = doc->NewElement("inertial");
    
    // Write mass
    XMLElement* mass = doc->NewElement("mass");
    mass->SetAttribute("value", inertial.mass);
    inertial_elem->InsertEndChild(mass);
    
    // Write origin
    write_pose(inertial_elem, inertial.origin, "origin");
    
    // Write inertia
    XMLElement* inertia = doc->NewElement("inertia");
    inertia->SetAttribute("ixx", inertial.ixx);
    inertia->SetAttribute("ixy", inertial.ixy);
    inertia->SetAttribute("ixz", inertial.ixz);
    inertia->SetAttribute("iyy", inertial.iyy);
    inertia->SetAttribute("iyz", inertial.iyz);
    inertia->SetAttribute("izz", inertial.izz);
    inertial_elem->InsertEndChild(inertia);
    
    parent->InsertEndChild(inertial_elem);
}

// ========== Helper: Write Visual ==========

void URDFParser::write_visual(XMLElement* parent, const URDFVisual& visual) {
    XMLDocument* doc = parent->GetDocument();
    XMLElement* visual_elem = doc->NewElement("visual");
    
    if (!visual.name.empty()) {
        visual_elem->SetAttribute("name", visual.name.c_str());
    }
    
    write_pose(visual_elem, visual.origin, "origin");
    write_geometry(visual_elem, visual.geometry);
    
    // Write material reference or inline
    if (!visual.material_name.empty()) {
        XMLElement* mat = doc->NewElement("material");
        mat->SetAttribute("name", visual.material_name.c_str());
        visual_elem->InsertEndChild(mat);
    } else if (!visual.material.name.empty()) {
        write_material(visual_elem, visual.material);
    }
    
    parent->InsertEndChild(visual_elem);
}

// ========== Helper: Write Collision ==========

void URDFParser::write_collision(XMLElement* parent, const URDFCollision& collision) {
    XMLDocument* doc = parent->GetDocument();
    XMLElement* collision_elem = doc->NewElement("collision");
    
    if (!collision.name.empty()) {
        collision_elem->SetAttribute("name", collision.name.c_str());
    }
    
    write_pose(collision_elem, collision.origin, "origin");
    write_geometry(collision_elem, collision.geometry);
    
    parent->InsertEndChild(collision_elem);
}

// ========== Helper: Write Link ==========

void URDFParser::write_link(XMLElement* parent, const URDFLink& link) {
    XMLDocument* doc = parent->GetDocument();
    XMLElement* link_elem = doc->NewElement("link");
    link_elem->SetAttribute("name", link.name.c_str());
    
    // Write inertial
    write_inertial(link_elem, link.inertial);
    
    // Write visuals
    for (const auto& visual : link.visuals) {
        write_visual(link_elem, visual);
    }
    
    // Write collisions
    for (const auto& collision : link.collisions) {
        write_collision(link_elem, collision);
    }
    
    parent->InsertEndChild(link_elem);
}

// ========== Helper: Write Joint ==========

void URDFParser::write_joint(XMLElement* parent, const URDFJoint& joint) {
    XMLDocument* doc = parent->GetDocument();
    XMLElement* joint_elem = doc->NewElement("joint");
    joint_elem->SetAttribute("name", joint.name.c_str());
    joint_elem->SetAttribute("type", joint_type_to_string(joint.type));
    
    // Write parent
    XMLElement* parent_elem = doc->NewElement("parent");
    parent_elem->SetAttribute("link", joint.parent_link.c_str());
    joint_elem->InsertEndChild(parent_elem);
    
    // Write child
    XMLElement* child_elem = doc->NewElement("child");
    child_elem->SetAttribute("link", joint.child_link.c_str());
    joint_elem->InsertEndChild(child_elem);
    
    // Write origin
    write_pose(joint_elem, joint.origin, "origin");
    
    // Write axis
    XMLElement* axis = doc->NewElement("axis");
    std::ostringstream xyz;
    xyz << joint.axis.x << " " << joint.axis.y << " " << joint.axis.z;
    axis->SetAttribute("xyz", xyz.str().c_str());
    joint_elem->InsertEndChild(axis);
    
    // Write limit (for revolute/prismatic)
    if (joint.type == URDFJointType::REVOLUTE || joint.type == URDFJointType::PRISMATIC) {
        XMLElement* limit = doc->NewElement("limit");
        limit->SetAttribute("lower", joint.lower_limit);
        limit->SetAttribute("upper", joint.upper_limit);
        limit->SetAttribute("effort", joint.effort_limit);
        limit->SetAttribute("velocity", joint.velocity_limit);
        joint_elem->InsertEndChild(limit);
    }
    
    // Write dynamics
    XMLElement* dynamics = doc->NewElement("dynamics");
    dynamics->SetAttribute("damping", joint.damping);
    dynamics->SetAttribute("friction", joint.friction);
    joint_elem->InsertEndChild(dynamics);
    
    parent->InsertEndChild(joint_elem);
}

} // namespace io
} // namespace basements
