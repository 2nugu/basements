#ifndef BASEMENTS_ENGINE_PLUGIN_INTERFACE_H
#define BASEMENTS_ENGINE_PLUGIN_INTERFACE_H

#include "basements/engine/core/quantity_storage.h"
#include "basements/engine/core/compute_graph.h"
#include <string>
#include <memory>

namespace basements {
namespace engine {

/// Base interface for physics plugins
class PhysicsPlugin {
public:
    virtual ~PhysicsPlugin() = default;

    /// Get plugin name
    virtual std::string name() const = 0;

    /// Get plugin version
    virtual std::string version() const = 0;

    /// Register quantities needed by this plugin
    virtual void register_quantities(QuantityStorage& storage) = 0;

    /// Setup compute graph for this plugin
    virtual void setup_compute_graph(ComputeGraph& graph) = 0;

    /// Update plugin state
    virtual void update(float delta_time_seconds) = 0;

    /// Initialize plugin
    virtual void initialize() {}

    /// Shutdown plugin
    virtual void shutdown() {}

    /// Get plugin description
    virtual std::string description() const { return ""; }
};

} // namespace engine
} // namespace basements

#endif // BASEMENTS_ENGINE_PLUGIN_INTERFACE_H
