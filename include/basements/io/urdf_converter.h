#pragma once
#include "basements/io/urdf_types.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"
#include "basements/engine/scene_graph.h"
#include <unordered_map>
#include <functional>

namespace basements {
namespace io {

using math::Vec3;
using math::Quaternion;

/**
 * @brief URDF ↔ SceneGraph Converter (Header-Only)
 * 
 * This is a header-only implementation to avoid circular dependencies.
 * Include this file only where you need SceneGraph conversion.
 */
class URDFConverter {
public:
    /**
     * @brief Convert URDF robot to SceneGraph
     */
    static bool to_scene_graph(const URDFRobot& robot,
                                editor::SceneGraph& scene_graph,
                                editor::NodeID& root_id,
                                std::string& error_message) {
        if (robot.links.empty()) {
            error_message = "Robot has no links";
            return false;
        }
        
        // Map link names to node IDs
        std::unordered_map<std::string, editor::NodeID> link_to_node;
        
        // Create root node for robot
        root_id = scene_graph.create_node(editor::NodeType::GROUP, robot.name);

        // Create nodes for all links
        for (const auto& link : robot.links) {
            editor::NodeID node_id = scene_graph.create_node(editor::NodeType::RIGID_BODY, link.name);
            link_to_node[link.name] = node_id;
            
            // Set mass and inertia
            editor::SceneNode* node = scene_graph.get_node(node_id);
            if (node) {
                node->physics.mass = link.inertial.mass;
                node->physics.inertia = Vec3(link.inertial.ixx, link.inertial.iyy, link.inertial.izz);
                
                // Process ALL collision elements (multi-geometry support)
                node->collision_elements.clear();
                for (const auto& collision : link.collisions) {
                    editor::SceneNode::CollisionElement elem;
                    
                    // Map geometry type
                    switch (collision.geometry.type) {
                        case URDFGeometry::BOX:
                            elem.shape = editor::ShapeType::BOX;
                            elem.size = Vec3(
                                collision.geometry.size.x,
                                collision.geometry.size.y,
                                collision.geometry.size.z
                            );
                            break;
                        case URDFGeometry::SPHERE:
                            elem.shape = editor::ShapeType::SPHERE;
                            elem.size = Vec3(
                                collision.geometry.radius,
                                collision.geometry.radius,
                                collision.geometry.radius
                            );
                            break;
                        case URDFGeometry::CYLINDER:
                            elem.shape = editor::ShapeType::CYLINDER;
                            elem.size = Vec3(
                                collision.geometry.radius,
                                collision.geometry.length,
                                collision.geometry.radius
                            );
                            break;
                        default:
                            elem.shape = editor::ShapeType::BOX;
                            elem.size = Vec3(1.0f, 1.0f, 1.0f);
                            break;
                    }
                    
                    // Set offset and rotation from collision origin
                    elem.offset = Vec3(
                        collision.origin.position.x,
                        collision.origin.position.y,
                        collision.origin.position.z
                    );
                    elem.rotation = Quaternion(
                        collision.origin.orientation.w,
                        collision.origin.orientation.x,
                        collision.origin.orientation.y,
                        collision.origin.orientation.z
                    );
                    
                    node->collision_elements.push_back(elem);
                }
                
                // Set primary collision (first element for backward compatibility)
                if (!node->collision_elements.empty()) {
                    node->physics.shape_type = node->collision_elements[0].shape;
                    node->physics.shape_size = node->collision_elements[0].size;
                }
                
                // Process ALL visual elements (multi-geometry support)
                node->visual_elements.clear();
                for (const auto& visual : link.visuals) {
                    editor::SceneNode::VisualElement elem;
                    if (visual.geometry.type == URDFGeometry::MESH) {
                        elem.mesh_path = visual.geometry.mesh_filename;
                    }
                    
                    elem.offset = Vec3(
                        visual.origin.position.x,
                        visual.origin.position.y,
                        visual.origin.position.z
                    );
                    
                    elem.rotation = Quaternion(
                        visual.origin.orientation.w,
                        visual.origin.orientation.x,
                        visual.origin.orientation.y,
                        visual.origin.orientation.z
                    );
                    
                    if (!visual.material.name.empty()) {
                        elem.material.albedo = Vec3(
                            visual.material.color.r,
                            visual.material.color.g,
                            visual.material.color.b
                        );
                    }
                    node->visual_elements.push_back(elem);
                }
                
                // Set backward-compatible main visual properties
                if (!node->visual_elements.empty()) {
                    node->material.albedo = node->visual_elements[0].material.albedo;
                }
            }
        }
        
        // Build hierarchy based on joints
        for (const auto& joint : robot.joints) {
            auto parent_it = link_to_node.find(joint.parent_link);
            auto child_it = link_to_node.find(joint.child_link);
            
            if (parent_it == link_to_node.end() || child_it == link_to_node.end()) {
                continue;
            }
            
            editor::NodeID parent_id = parent_it->second;
            editor::NodeID child_id = child_it->second;
            
            scene_graph.set_parent(child_id, parent_id);

            // Set transform from joint origin
            editor::SceneNode* child_node = scene_graph.get_node(child_id);
            if (child_node) {
                child_node->local_position = Vec3(
                    joint.origin.position.x,
                    joint.origin.position.y,
                    joint.origin.position.z
                );

                child_node->local_rotation = Quaternion(
                    joint.origin.orientation.w,
                    joint.origin.orientation.x,
                    joint.origin.orientation.y,
                    joint.origin.orientation.z
                );
            }

            // Also create a NodeType::JOINT scene node so EditorApp::play()
            // (which wires JOINT nodes to PhysicsWorldGPU constraints) sees
            // the URDF joints. FLOATING / PLANAR have no rigid analogue and
            // are skipped (the child link is left as a free 6-DOF body).
            using basements::dynamics::JointType;
            bool has_constraint = true;
            JointType jt = JointType::FIXED;
            switch (joint.type) {
                case URDFJointType::REVOLUTE:   jt = JointType::HINGE;  break;
                case URDFJointType::CONTINUOUS: jt = JointType::HINGE;  break;
                case URDFJointType::PRISMATIC:  jt = JointType::SLIDER; break;
                case URDFJointType::FIXED:      jt = JointType::FIXED;  break;
                case URDFJointType::FLOATING:
                case URDFJointType::PLANAR:
                    has_constraint = false;
                    break;
            }
            if (has_constraint) {
                editor::NodeID joint_node_id =
                    scene_graph.create_node(editor::NodeType::JOINT, joint.name);
                editor::SceneNode* jn = scene_graph.get_node(joint_node_id);
                if (jn) {
                    jn->joint.joint_type        = jt;
                    jn->joint.connected_body_a  = parent_id;
                    jn->joint.connected_body_b  = child_id;
                    jn->joint.anchor_a          = Vec3(joint.origin.position.x,
                                                       joint.origin.position.y,
                                                       joint.origin.position.z);
                    jn->joint.anchor_b          = Vec3::zero();
                    jn->joint.axis_a            = Vec3(joint.axis.x, joint.axis.y, joint.axis.z);
                    jn->joint.axis_b            = jn->joint.axis_a;
                    const bool revolute_or_prismatic =
                        (joint.type == URDFJointType::REVOLUTE ||
                         joint.type == URDFJointType::PRISMATIC);
                    jn->joint.enable_limits = revolute_or_prismatic;
                    jn->joint.lower_limit   = joint.lower_limit;
                    jn->joint.upper_limit   = joint.upper_limit;
                }
            }
        }
        
        // Attach all root links to robot root
        for (const auto& link : robot.links) {
            bool is_child = false;
            for (const auto& joint : robot.joints) {
                if (joint.child_link == link.name) {
                    is_child = true;
                    break;
                }
            }
            
            if (!is_child) {
                editor::NodeID link_id = link_to_node[link.name];
                scene_graph.set_parent(link_id, root_id);
            }
        }
        
        return true;
    }
    
    /**
     * @brief Convert SceneGraph to URDF robot
     */
    static bool from_scene_graph(const editor::SceneGraph& scene_graph,
                                  editor::NodeID root_id,
                                  URDFRobot& robot,
                                  std::string& error_message) {
        const editor::SceneNode* root = scene_graph.get_node(root_id);
        if (!root) {
            error_message = "Invalid root node ID";
            return false;
        }
        
        robot.name = root->name;
        robot.links.clear();
        robot.joints.clear();
        
        // Traverse scene graph
        std::function<void(editor::NodeID, const std::string&)> traverse;
        traverse = [&](editor::NodeID node_id, const std::string& parent_link_name) {
            const editor::SceneNode* node = scene_graph.get_node(node_id);
            if (!node) return;
            
            if (node->type != editor::NodeType::RIGID_BODY) {
                for (editor::NodeID child_id : node->children) {
                    traverse(child_id, parent_link_name);
                }
                return;
            }
            
            // Create URDF link
            URDFLink link(node->name);
            link.inertial.mass = node->physics.mass;
            link.inertial.ixx = node->physics.inertia.x;
            link.inertial.iyy = node->physics.inertia.y;
            link.inertial.izz = node->physics.inertia.z;
            
            // Add Visual elements
            if (node->visual_elements.empty()) {
                // Backward compatibility: create one from basic properties
                URDFVisual visual;
                visual.geometry.type = URDFGeometry::BOX;
                visual.geometry.size = URDFVector3(
                    node->physics.shape_size.x,
                    node->physics.shape_size.y,
                    node->physics.shape_size.z
                );
                visual.material.color = URDFColor(
                    node->material.albedo.x,
                    node->material.albedo.y,
                    node->material.albedo.z
                );
                link.visuals.push_back(visual);
            } else {
                for (const auto& velem : node->visual_elements) {
                    URDFVisual visual;
                    if (!velem.mesh_path.empty()) {
                        visual.geometry.type = URDFGeometry::MESH;
                        visual.geometry.mesh_filename = velem.mesh_path;
                        visual.geometry.mesh_scale = URDFVector3(1.0f, 1.0f, 1.0f);
                    } else {
                        visual.geometry.type = URDFGeometry::BOX;
                        visual.geometry.size = URDFVector3(node->physics.shape_size.x, node->physics.shape_size.y, node->physics.shape_size.z);
                    }
                    visual.origin.position = URDFVector3(velem.offset.x, velem.offset.y, velem.offset.z);
                    visual.origin.orientation = URDFQuaternion(velem.rotation.w, velem.rotation.x, velem.rotation.y, velem.rotation.z);
                    visual.material.color = URDFColor(velem.material.albedo.x, velem.material.albedo.y, velem.material.albedo.z);
                    link.visuals.push_back(visual);
                }
            }
            
            // Add Collision elements
            if (node->collision_elements.empty()) {
                // Backward compatibility
                URDFCollision collision;
                collision.geometry.type = URDFGeometry::BOX;
                collision.geometry.size = URDFVector3(
                    node->physics.shape_size.x,
                    node->physics.shape_size.y,
                    node->physics.shape_size.z
                );
                link.collisions.push_back(collision);
            } else {
                for (const auto& celem : node->collision_elements) {
                    URDFCollision collision;
                    switch (celem.shape) {
                        case editor::ShapeType::BOX:
                            collision.geometry.type = URDFGeometry::BOX;
                            collision.geometry.size = URDFVector3(celem.size.x, celem.size.y, celem.size.z);
                            break;
                        case editor::ShapeType::SPHERE:
                            collision.geometry.type = URDFGeometry::SPHERE;
                            collision.geometry.radius = celem.size.x;
                            break;
                        case editor::ShapeType::CYLINDER:
                        case editor::ShapeType::CAPSULE:
                            collision.geometry.type = URDFGeometry::CYLINDER;
                            collision.geometry.radius = celem.size.x;
                            collision.geometry.length = celem.size.y;
                            break;
                        default:
                            collision.geometry.type = URDFGeometry::BOX;
                            collision.geometry.size = URDFVector3(celem.size.x, celem.size.y, celem.size.z);
                            break;
                    }
                    collision.origin.position = URDFVector3(celem.offset.x, celem.offset.y, celem.offset.z);
                    collision.origin.orientation = URDFQuaternion(celem.rotation.w, celem.rotation.x, celem.rotation.y, celem.rotation.z);
                    link.collisions.push_back(collision);
                }
            }
            
            robot.links.push_back(link);
            
            // Create joint if has parent
            if (!parent_link_name.empty()) {
                URDFJoint joint(node->name + "_joint");
                joint.type = URDFJointType::FIXED;
                joint.parent_link = parent_link_name;
                joint.child_link = node->name;
                joint.origin.position = URDFVector3(
                    node->local_position.x,
                    node->local_position.y,
                    node->local_position.z
                );
                joint.origin.orientation = URDFQuaternion(
                    node->local_rotation.w,
                    node->local_rotation.x,
                    node->local_rotation.y,
                    node->local_rotation.z
                );
                robot.joints.push_back(joint);
            }
            
            // Recurse
            for (editor::NodeID child_id : node->children) {
                traverse(child_id, node->name);
            }
        };
        
        for (editor::NodeID child_id : root->children) {
            traverse(child_id, "");
        }
        
        return true;
    }
};

} // namespace io
} // namespace basements
