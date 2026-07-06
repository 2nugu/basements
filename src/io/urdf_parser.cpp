#include "basements/io/urdf_parser.h"
#include <tinyxml2.h>
#include <cmath>
#include <sstream>
#include <fstream>

using namespace tinyxml2;

namespace basements {
namespace io {

// ========== Parsing ==========

URDFParseResult URDFParser::parse_file(const std::string& filepath) {
    XMLDocument doc;
    XMLError err = doc.LoadFile(filepath.c_str());
    
    if (err != XML_SUCCESS) {
        last_error_ = "Failed to load file: " + filepath;
        return URDFParseResult(false, last_error_);
    }
    
    return parse_document(doc);
}

URDFParseResult URDFParser::parse_string(const std::string& urdf_xml) {
    XMLDocument doc;
    XMLError err = doc.Parse(urdf_xml.c_str());
    
    if (err != XML_SUCCESS) {
        last_error_ = "Failed to parse XML string";
        return URDFParseResult(false, last_error_);
    }
    
    return parse_document(doc);
}

URDFParseResult URDFParser::parse_document(tinyxml2::XMLDocument& doc) {
    URDFParseResult result;
    result.success = true;
    
    // Find <robot> root element
    XMLElement* robot_elem = doc.FirstChildElement("robot");
    if (!robot_elem) {
        result.success = false;
        result.error_message = "No <robot> element found";
        return result;
    }
    
    // Parse robot name
    const char* name = robot_elem->Attribute("name");
    result.robot.name = name ? name : "unnamed_robot";
    
    // Parse all <link> elements
    for (XMLElement* link_elem = robot_elem->FirstChildElement("link");
         link_elem;
         link_elem = link_elem->NextSiblingElement("link")) {
        result.robot.links.push_back(parse_link(link_elem));
    }
    
    // Parse all <joint> elements
    for (XMLElement* joint_elem = robot_elem->FirstChildElement("joint");
         joint_elem;
         joint_elem = joint_elem->NextSiblingElement("joint")) {
        result.robot.joints.push_back(parse_joint(joint_elem));
    }
    
    // Parse all <material> elements (global)
    for (XMLElement* material_elem = robot_elem->FirstChildElement("material");
         material_elem;
         material_elem = material_elem->NextSiblingElement("material")) {
        result.robot.materials.push_back(parse_material(material_elem));
    }
    
    return result;
}

// ========== Helper: Parse Vector3 ==========

URDFVector3 URDFParser::parse_vector3(const char* str) {
    if (!str) return URDFVector3();
    
    std::istringstream iss(str);
    URDFVector3 v;
    iss >> v.x >> v.y >> v.z;
    return v;
}

// ========== Helper: Parse Quaternion from RPY ==========

URDFQuaternion URDFParser::parse_quaternion(const char* rpy_str) {
    if (!rpy_str) return URDFQuaternion();  // Identity
    
    // Parse roll-pitch-yaw
    std::istringstream iss(rpy_str);
    float roll, pitch, yaw;
    iss >> roll >> pitch >> yaw;
    
    // Convert RPY to quaternion
    float cy = std::cos(yaw * 0.5f);
    float sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);
    
    URDFQuaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    
    return q;
}

// ========== Helper: Parse Pose ==========

URDFPose URDFParser::parse_pose(tinyxml2::XMLElement* elem) {
    URDFPose pose;
    
    if (!elem) return pose;
    
    // Parse xyz attribute
    const char* xyz = elem->Attribute("xyz");
    if (xyz) {
        pose.position = parse_vector3(xyz);
    }
    
    // Parse rpy attribute
    const char* rpy = elem->Attribute("rpy");
    if (rpy) {
        pose.orientation = parse_quaternion(rpy);
    }
    
    return pose;
}

// ========== Helper: Parse Geometry ==========

URDFGeometry URDFParser::parse_geometry(tinyxml2::XMLElement* elem) {
    URDFGeometry geom;
    
    if (!elem) return geom;
    
    // Check geometry type
    if (XMLElement* box = elem->FirstChildElement("box")) {
        geom.type = URDFGeometry::BOX;
        geom.size = parse_vector3(box->Attribute("size"));
    }
    else if (XMLElement* sphere = elem->FirstChildElement("sphere")) {
        geom.type = URDFGeometry::SPHERE;
        sphere->QueryFloatAttribute("radius", &geom.radius);
    }
    else if (XMLElement* cylinder = elem->FirstChildElement("cylinder")) {
        geom.type = URDFGeometry::CYLINDER;
        cylinder->QueryFloatAttribute("radius", &geom.radius);
        cylinder->QueryFloatAttribute("length", &geom.length);
    }
    else if (XMLElement* mesh = elem->FirstChildElement("mesh")) {
        geom.type = URDFGeometry::MESH;
        const char* filename = mesh->Attribute("filename");
        if (filename) geom.mesh_filename = filename;
        
        const char* scale = mesh->Attribute("scale");
        if (scale) geom.mesh_scale = parse_vector3(scale);
    }
    
    return geom;
}

// ========== Helper: Parse Material ==========

URDFMaterial URDFParser::parse_material(tinyxml2::XMLElement* elem) {
    URDFMaterial material;
    
    if (!elem) return material;
    
    const char* name = elem->Attribute("name");
    if (name) material.name = name;
    
    // Parse color
    if (XMLElement* color = elem->FirstChildElement("color")) {
        const char* rgba = color->Attribute("rgba");
        if (rgba) {
            std::istringstream iss(rgba);
            iss >> material.color.r >> material.color.g 
                >> material.color.b >> material.color.a;
        }
    }
    
    // Parse texture
    if (XMLElement* texture = elem->FirstChildElement("texture")) {
        const char* filename = texture->Attribute("filename");
        if (filename) material.texture_filename = filename;
    }
    
    return material;
}

// ========== Helper: Parse Inertial ==========

URDFInertial URDFParser::parse_inertial(tinyxml2::XMLElement* elem) {
    URDFInertial inertial;
    
    if (!elem) return inertial;
    
    // Parse mass
    if (XMLElement* mass = elem->FirstChildElement("mass")) {
        mass->QueryFloatAttribute("value", &inertial.mass);
    }
    
    // Parse origin
    if (XMLElement* origin = elem->FirstChildElement("origin")) {
        inertial.origin = parse_pose(origin);
    }
    
    // Parse inertia tensor
    if (XMLElement* inertia = elem->FirstChildElement("inertia")) {
        inertia->QueryFloatAttribute("ixx", &inertial.ixx);
        inertia->QueryFloatAttribute("ixy", &inertial.ixy);
        inertia->QueryFloatAttribute("ixz", &inertial.ixz);
        inertia->QueryFloatAttribute("iyy", &inertial.iyy);
        inertia->QueryFloatAttribute("iyz", &inertial.iyz);
        inertia->QueryFloatAttribute("izz", &inertial.izz);
    }
    
    return inertial;
}

// ========== Helper: Parse Visual ==========

URDFVisual URDFParser::parse_visual(tinyxml2::XMLElement* elem) {
    URDFVisual visual;
    
    if (!elem) return visual;
    
    const char* name = elem->Attribute("name");
    if (name) visual.name = name;
    
    // Parse origin
    if (XMLElement* origin = elem->FirstChildElement("origin")) {
        visual.origin = parse_pose(origin);
    }
    
    // Parse geometry
    if (XMLElement* geometry = elem->FirstChildElement("geometry")) {
        visual.geometry = parse_geometry(geometry);
    }
    
    // Parse material
    if (XMLElement* material = elem->FirstChildElement("material")) {
        const char* mat_name = material->Attribute("name");
        if (mat_name) {
            visual.material_name = mat_name;
        }
        // Parse inline material
        visual.material = parse_material(material);
    }
    
    return visual;
}

// ========== Helper: Parse Collision ==========

URDFCollision URDFParser::parse_collision(tinyxml2::XMLElement* elem) {
    URDFCollision collision;
    
    if (!elem) return collision;
    
    const char* name = elem->Attribute("name");
    if (name) collision.name = name;
    
    // Parse origin
    if (XMLElement* origin = elem->FirstChildElement("origin")) {
        collision.origin = parse_pose(origin);
    }
    
    // Parse geometry
    if (XMLElement* geometry = elem->FirstChildElement("geometry")) {
        collision.geometry = parse_geometry(geometry);
    }
    
    return collision;
}

// ========== Helper: Parse Link ==========

URDFLink URDFParser::parse_link(tinyxml2::XMLElement* elem) {
    URDFLink link;
    
    const char* name = elem->Attribute("name");
    if (name) link.name = name;
    
    // Parse inertial
    if (XMLElement* inertial = elem->FirstChildElement("inertial")) {
        link.inertial = parse_inertial(inertial);
    }
    
    // Parse all visual elements
    for (XMLElement* visual = elem->FirstChildElement("visual");
         visual;
         visual = visual->NextSiblingElement("visual")) {
        link.visuals.push_back(parse_visual(visual));
    }
    
    // Parse all collision elements
    for (XMLElement* collision = elem->FirstChildElement("collision");
         collision;
         collision = collision->NextSiblingElement("collision")) {
        link.collisions.push_back(parse_collision(collision));
    }
    
    return link;
}

// ========== Helper: Parse Joint ==========

URDFJoint URDFParser::parse_joint(tinyxml2::XMLElement* elem) {
    URDFJoint joint;
    
    const char* name = elem->Attribute("name");
    if (name) joint.name = name;
    
    const char* type = elem->Attribute("type");
    if (type) joint.type = string_to_joint_type(type);
    
    // Parse parent link
    if (XMLElement* parent = elem->FirstChildElement("parent")) {
        const char* link = parent->Attribute("link");
        if (link) joint.parent_link = link;
    }
    
    // Parse child link
    if (XMLElement* child = elem->FirstChildElement("child")) {
        const char* link = child->Attribute("link");
        if (link) joint.child_link = link;
    }
    
    // Parse origin
    if (XMLElement* origin = elem->FirstChildElement("origin")) {
        joint.origin = parse_pose(origin);
    }
    
    // Parse axis
    if (XMLElement* axis = elem->FirstChildElement("axis")) {
        const char* xyz = axis->Attribute("xyz");
        if (xyz) joint.axis = parse_vector3(xyz);
    }
    
    // Parse limit
    if (XMLElement* limit = elem->FirstChildElement("limit")) {
        limit->QueryFloatAttribute("lower", &joint.lower_limit);
        limit->QueryFloatAttribute("upper", &joint.upper_limit);
        limit->QueryFloatAttribute("effort", &joint.effort_limit);
        limit->QueryFloatAttribute("velocity", &joint.velocity_limit);
    }
    
    // Parse dynamics
    if (XMLElement* dynamics = elem->FirstChildElement("dynamics")) {
        dynamics->QueryFloatAttribute("damping", &joint.damping);
        dynamics->QueryFloatAttribute("friction", &joint.friction);
    }
    
    return joint;
}

} // namespace io
} // namespace basements
