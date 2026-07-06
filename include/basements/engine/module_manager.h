#ifndef BASEMENTS_EDITOR_MODULE_MANAGER_H
#define BASEMENTS_EDITOR_MODULE_MANAGER_H

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

namespace basements {
namespace editor {

class Workspace;

/**
 * @brief Base interface for all editor modules
 */
class IModule {
public:
    virtual ~IModule() = default;
    
    virtual std::string name() const = 0;
    virtual void on_init(Workspace& workspace) = 0;
    virtual void on_update(float delta_time) = 0;
    virtual void on_shutdown() = 0;
    
    // UI rendering (optional)
    virtual void on_render_ui() {}
};

/**
 * @brief Manages the lifecycle and interaction of editor modules
 */
class ModuleManager {
public:
    ModuleManager(Workspace& workspace);
    ~ModuleManager();

    // Registration
    template<typename T, typename... Args>
    void add_module(Args&&... args) {
        auto module = std::make_unique<T>(std::forward<Args>(args)...);
        std::string module_name = module->name();
        module->on_init(workspace_);
        modules_[module_name] = std::move(module);
    }

    // Lifecycle
    void update(float delta_time);
    void render_ui();
    void shutdown();

    // Access
    IModule* get_module(const std::string& name);

private:
    Workspace& workspace_;
    std::unordered_map<std::string, std::unique_ptr<IModule>> modules_;
};

} // namespace editor
} // namespace basements

#endif // BASEMENTS_EDITOR_MODULE_MANAGER_H
