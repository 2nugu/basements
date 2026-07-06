#ifndef BASEMENTS_EDITOR_PHYSICS_PANEL_H
#define BASEMENTS_EDITOR_PHYSICS_PANEL_H

#include <string>

namespace basements { namespace mpm { class MPMSolver; } }

namespace basements {
namespace editor {

class PhysicsPanel {
public:
    PhysicsPanel();
    ~PhysicsPanel();

    void on_render(bool* open);

    // Called from EditorApp main loop at the correct times
    void step(float dt);
    void render_particles();

    // Exposed for cross-system coupling (e.g. MPM ↔ rigid body Dirichlet BC).
    // Returns nullptr if no MPM solver is allocated yet.
    basements::mpm::MPMSolver* get_solver();

private:
    struct Impl;
    // PIMPL strictly to avoid including ImGui headers in the public header
    // if we wanted to be strict, but for simplicity in editor code, 
    // we often include imgui in headers. 
    // However, to follow the pattern set by EditorApp, let's use PIMPL or just implementation in cpp.
    // For Panels, it's often easier to just have the render function.
    // But we need state (search filter, current env manager).
    
    // Using a pointer to implementation to keep header clean
    struct Impl;
    Impl* impl_;
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_PHYSICS_PANEL_H
