#ifndef BASEMENTS_EDITOR_COMMAND_HISTORY_H
#define BASEMENTS_EDITOR_COMMAND_HISTORY_H

/**
 * @file command_history.h
 * @brief Undo/Redo system for editor operations
 */

#include <string>
#include <vector>
#include <memory>
#include <stack>
#include "basements/engine/scene_graph.h"
#include "basements/core/math/vec3.h"
#include "basements/core/math/quaternion.h"

namespace basements {
namespace editor {

// ============================================================================
// Command Interface
// ============================================================================
class Command {
public:
    virtual ~Command() = default;
    virtual void execute() = 0;
    virtual void undo() = 0;
    virtual std::string get_name() const = 0;
};

// ============================================================================
// Concrete Commands
// ============================================================================

// Create Node Command
class CreateNodeCommand : public Command {
public:
    CreateNodeCommand(SceneGraph* scene_graph, NodeType type, const std::string& name);
    CreateNodeCommand(SceneGraph* scene_graph, const std::string& name, ShapeType shape); // For bodies
    
    void execute() override;
    void undo() override;
    std::string get_name() const override { return "Create Node"; }

private:
    SceneGraph* scene_graph_;
    NodeType type_;
    std::string name_;
    ShapeType shape_type_ = ShapeType::BOX;
    NodeID created_node_id_ = INVALID_NODE_ID;
    bool is_body_ = false;
};

// Delete Node Command
class DeleteNodeCommand : public Command {
public:
    DeleteNodeCommand(SceneGraph* scene_graph, NodeID node_id);
    
    void execute() override;
    void undo() override;
    std::string get_name() const override { return "Delete Node"; }

private:
    SceneGraph* scene_graph_;
    NodeID node_id_;
    std::unique_ptr<SceneNode> backup_node_; // Stores the deleted node
};

// Change Transform Command
class ChangeTransformCommand : public Command {
public:
    ChangeTransformCommand(SceneGraph* scene_graph, NodeID node_id, 
                          const basements::math::Vec3& old_pos, const basements::math::Vec3& new_pos,
                          const basements::math::Quaternion& old_rot, const basements::math::Quaternion& new_rot,
                          const basements::math::Vec3& old_scale, const basements::math::Vec3& new_scale);
                          
    void execute() override;
    void undo() override;
    std::string get_name() const override { return "Transform Change"; }

private:
    SceneGraph* scene_graph_;
    NodeID node_id_;
    basements::math::Vec3 old_pos_;
    basements::math::Vec3 new_pos_;
    basements::math::Quaternion old_rot_;
    basements::math::Quaternion new_rot_;
    basements::math::Vec3 old_scale_;
    basements::math::Vec3 new_scale_;
};

// Reparent Node Command
class ReparentNodeCommand : public Command {
public:
    ReparentNodeCommand(SceneGraph* scene_graph, NodeID node_id, NodeID new_parent_id);
    
    void execute() override;
    void undo() override;
    std::string get_name() const override { return "Reparent Node"; }

private:
    SceneGraph* scene_graph_;
    NodeID node_id_;
    NodeID new_parent_id_;
    NodeID old_parent_id_;
    // We might need to handle local transform preservation?
    // For MVP, just reparent. Transforms are typically relative to parent. 
    // If we want to keep world position, we need to recalculate local transform.
    // Let's assume SceneGraph handles or we do simple reparent.
    // Users usually expect "Keep World Transform" behavior.
    
    basements::math::Vec3 old_local_pos_;
    basements::math::Quaternion old_local_rot_;
    basements::math::Vec3 old_local_scale_;
    
    basements::math::Vec3 new_local_pos_;
    basements::math::Quaternion new_local_rot_;
    basements::math::Vec3 new_local_scale_;
};

// Duplicate Node Command
class DuplicateNodeCommand : public Command {
public:
    DuplicateNodeCommand(SceneGraph* scene_graph, NodeID original_node_id);

    void execute() override;
    void undo() override;
    std::string get_name() const override { return "Duplicate Node"; }

private:
    SceneGraph* scene_graph_;
    NodeID original_node_id_;
    NodeID created_node_id_ = INVALID_NODE_ID;
};

// Composite Command (Batch)
class CompositeCommand : public Command {
public:
    void add_command(std::unique_ptr<Command> cmd);
    
    void execute() override;
    void undo() override;
    std::string get_name() const override { return "Batch Command"; }
    bool is_empty() const { return commands_.empty(); }

private:
    std::vector<std::unique_ptr<Command>> commands_;
};

// ============================================================================
// Command History (Manager)
// ============================================================================
class CommandHistory {
public:
    void execute_command(std::unique_ptr<Command> cmd);
    void undo();
    void redo();
    
    bool can_undo() const;
    bool can_redo() const;
    
    void clear();

private:
    std::stack<std::unique_ptr<Command>> undo_stack_;
    std::stack<std::unique_ptr<Command>> redo_stack_;
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_COMMAND_HISTORY_H
