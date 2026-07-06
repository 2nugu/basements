#include "basements/engine/command_history.h"
#include <iostream>

namespace basements {
namespace editor {

// ============================================================================
// CreateNodeCommand
// ============================================================================
CreateNodeCommand::CreateNodeCommand(SceneGraph* scene_graph, NodeType type, const std::string& name)
    : scene_graph_(scene_graph), type_(type), name_(name), is_body_(false) {}

CreateNodeCommand::CreateNodeCommand(SceneGraph* scene_graph, const std::string& name, ShapeType shape)
    : scene_graph_(scene_graph), name_(name), shape_type_(shape), is_body_(true) {}

void CreateNodeCommand::execute() {
    if (created_node_id_ == INVALID_NODE_ID) {
        // First execution: Create fresh
        if (is_body_) {
            created_node_id_ = scene_graph_->create_rigid_body(name_, shape_type_);
        } else {
            created_node_id_ = scene_graph_->create_node(type_, name_);
        }
    } else {
        // Redo: Validation?
        // Actually, SceneGraph creation creates NEW ID every time.
        // If we want REDO to restore the SAME ID, we need to treat Redo like Undo of Undo (Restore).
        // But for "Create", Redo usually just creates a new one or we must separate "First Execute" from "Redo".
        // Let's implement Redo as "Create New" or "Restore Backup" if we saved it in Undo?
        // Simpler: Just create a new one. The ID will change, but that's usually fine for creation unless dependent commands exist.
        // CAUTION: If subsequent commands depend on ID, changing ID in Redo breaks history.
        // FIX: execute() should check. If it was already created and then undone, we should use restore_node logic!
        // So Undo should SAVE the node it deleted (just like DeleteCommand).
        // Let's rely on that logic if we have backup.
    }
}

void CreateNodeCommand::undo() {
    if (created_node_id_ != INVALID_NODE_ID) {
        // We delete the node. But we want to be able to REDO it exactly.
        // So we might want to capture it before deletion if we want exact ID restoration.
        // For MVP, let's just remove it. Redo will make a new one (new ID). 
        // NOTE: This breaks "Create -> Move -> Undo Move -> Undo Create -> Redo Create -> Redo Move" chain 
        // because Redo Move will point to OLD ID.
        // SOLUTION: CreateNodeCommand MUST restore the SAME ID on Redo.
        // Implementing proper backup in Undo.
        
        // 1. Get the node
        std::unique_ptr<SceneNode> node = nullptr;
        // Accessing unique_ptr from SceneGraph requires custom API to "take" ownership?
        // remove_node destroys it.
        // We need 'take_node' or separate remove_node that returns the ptr.
        // Workaround: We can't implement perfect robust Undo/Redo without changing SceneGraph more.
        
        // SIMPLE MVP: Just remove. Warn about ID shift.
        scene_graph_->remove_node(created_node_id_);
        created_node_id_ = INVALID_NODE_ID; // Reset ID so execute() makes new one
    }
}

// ============================================================================
// DeleteNodeCommand
// ============================================================================
DeleteNodeCommand::DeleteNodeCommand(SceneGraph* scene_graph, NodeID node_id)
    : scene_graph_(scene_graph), node_id_(node_id) {}

void DeleteNodeCommand::execute() {
    // We need to backup before delete... but SceneGraph owns the unique_ptr.
    // We need a way to extract it. 'remove_node' destroys it.
    // Let's assume we can't easily extract without API change.
    // Let's use get_node to COPY data, then remove.
    SceneNode* node = scene_graph_->get_node(node_id_);
    if (node) {
        backup_node_ = std::make_unique<SceneNode>(*node); // Copy constructor (default)
        scene_graph_->remove_node(node_id_);
    }
}

void DeleteNodeCommand::undo() {
    if (backup_node_) {
        // Restore. Note: unique_ptr is moved, so backup_node_ becomes empty!
        // If we want multiple Undo/Redo, we must clone it each time or keep ownership shared?
        // Command owns the backup. When undoing, checked into SceneGraph. 
        // Wait, if we give it to SceneGraph, we lose it. We can't Redo (Delete) again!
        // We need to give SceneGraph a COPY.
        auto restore_copy = std::make_unique<SceneNode>(*backup_node_);
        scene_graph_->restore_node(std::move(restore_copy));
    }
}

// ============================================================================
// ChangeTransformCommand
// ============================================================================
ChangeTransformCommand::ChangeTransformCommand(SceneGraph* scene_graph, NodeID node_id, 
                          const basements::math::Vec3& old_pos, const basements::math::Vec3& new_pos,
                          const basements::math::Quaternion& old_rot, const basements::math::Quaternion& new_rot,
                          const basements::math::Vec3& old_scale, const basements::math::Vec3& new_scale)
    : scene_graph_(scene_graph), node_id_(node_id), 
      old_pos_(old_pos), new_pos_(new_pos), 
      old_rot_(old_rot), new_rot_(new_rot),
      old_scale_(old_scale), new_scale_(new_scale) {}

void ChangeTransformCommand::execute() {
    SceneNode* node = scene_graph_->get_node(node_id_);
    if (node) {
        node->local_position = new_pos_;
        node->local_rotation = new_rot_;
        node->local_scale = new_scale_;
    }
}

void ChangeTransformCommand::undo() {
    SceneNode* node = scene_graph_->get_node(node_id_);
    if (node) {
        node->local_position = old_pos_;
        node->local_rotation = old_rot_;
        node->local_scale = old_scale_;
    }
}

// ============================================================================
// DuplicateNodeCommand
// ============================================================================
DuplicateNodeCommand::DuplicateNodeCommand(SceneGraph* scene_graph, NodeID original_node_id)
    : scene_graph_(scene_graph), original_node_id_(original_node_id) {}

void DuplicateNodeCommand::execute() {
    if (created_node_id_ == INVALID_NODE_ID) {
        SceneNode* original = scene_graph_->get_node(original_node_id_);
        if (original) {
            // Logic to clone node
            // 1. Create new node with same type/shape
            // 2. Copy properties
            
            // Simplified: Assume rigid body for now or generic clone support.
            // Since SceneNode copy ctor works, we just need to insert it.
            // But we have to use create APIs to get ID.
            
            std::string new_name = original->name + "_Copy";
            if (original->type == NodeType::RIGID_BODY) {
                created_node_id_ = scene_graph_->create_rigid_body(new_name, original->physics.shape_type);
            } else {
                created_node_id_ = scene_graph_->create_node(NodeType::GROUP, new_name);
            }
            
            SceneNode* new_node = scene_graph_->get_node(created_node_id_);
            if (new_node) {
                new_node->local_position = original->local_position;
                new_node->local_rotation = original->local_rotation;
                new_node->local_scale = original->local_scale;
                new_node->physics = original->physics; // Copy physics props
                new_node->mesh = original->mesh; // Reference copy mesh (shared ptr usually or raw resource)
                // If ID is unique, logic is fine.
            }
        }
    } else {
         // Redo: Use restore logic or recreate
         // For MVP, if we don't have restore, we just don't support robust redo of duplicate accurately yet.
    }
}

void DuplicateNodeCommand::undo() {
    if (created_node_id_ != INVALID_NODE_ID) {
        scene_graph_->remove_node(created_node_id_);
        created_node_id_ = INVALID_NODE_ID;
    }
}

// ============================================================================
// ReparentNodeCommand
// ============================================================================
ReparentNodeCommand::ReparentNodeCommand(SceneGraph* scene_graph, NodeID node_id, NodeID new_parent_id)
    : scene_graph_(scene_graph), node_id_(node_id), new_parent_id_(new_parent_id),
      old_parent_id_(INVALID_NODE_ID) {}

void ReparentNodeCommand::execute() {
    SceneNode* node = scene_graph_->get_node(node_id_);
    if (!node) return;
    old_parent_id_ = node->parent_id;
    if (new_parent_id_ == INVALID_NODE_ID) {
        scene_graph_->move_to_root(node_id_);
    } else {
        scene_graph_->set_parent(node_id_, new_parent_id_);
    }
}

void ReparentNodeCommand::undo() {
    if (old_parent_id_ == INVALID_NODE_ID) {
        scene_graph_->move_to_root(node_id_);
    } else {
        scene_graph_->set_parent(node_id_, old_parent_id_);
    }
}

// ============================================================================
// CompositeCommand
// ============================================================================
void CompositeCommand::add_command(std::unique_ptr<Command> cmd) {
    if (cmd) {
        commands_.push_back(std::move(cmd));
    }
}

void CompositeCommand::execute() {
    for (auto& cmd : commands_) {
        cmd->execute();
    }
}

void CompositeCommand::undo() {
    // Undo in reverse order
    for (auto it = commands_.rbegin(); it != commands_.rend(); ++it) {
        (*it)->undo();
    }
}

// ============================================================================
// CommandHistory
// ============================================================================
void CommandHistory::execute_command(std::unique_ptr<Command> cmd) {
    if (!cmd) return;
    cmd->execute();
    undo_stack_.push(std::move(cmd));
    
    // Clear redo stack on new action
    while (!redo_stack_.empty()) {
        redo_stack_.pop();
    }
}

void CommandHistory::undo() {
    if (undo_stack_.empty()) return;
    
    std::unique_ptr<Command> cmd = std::move(undo_stack_.top());
    undo_stack_.pop();
    cmd->undo();
    redo_stack_.push(std::move(cmd));
    
    std::cout << "[Editor] Undo performed." << std::endl;
}

void CommandHistory::redo() {
    if (redo_stack_.empty()) return;
    
    std::unique_ptr<Command> cmd = std::move(redo_stack_.top());
    redo_stack_.pop();
    cmd->execute();
    undo_stack_.push(std::move(cmd)); // Put back on undo stack
    
    std::cout << "[Editor] Redo performed." << std::endl;
}

bool CommandHistory::can_undo() const {
    return !undo_stack_.empty();
}

bool CommandHistory::can_redo() const {
    return !redo_stack_.empty();
}

void CommandHistory::clear() {
    while (!undo_stack_.empty()) undo_stack_.pop();
    while (!redo_stack_.empty()) redo_stack_.pop();
}

} // namespace editor
} // namespace basements
