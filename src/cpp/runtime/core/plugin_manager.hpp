#pragma once

#include <memory>
#include <string>
#include <vector>

#include "core/plugin.hpp"
#include "core/plugin_registry.hpp"
#include "core/runtime_config.hpp"

namespace robot_runtime {

class PluginManager {
 public:
  explicit PluginManager(const PluginRegistry& registry);

  void Load(const RuntimeConfig& config, const PluginContext& context);
  void Start();
  void Stop();

 private:
  struct LoadedPlugin {
    PluginSpec spec;
    std::unique_ptr<Plugin> plugin;
    bool started = false;
  };

  const PluginRegistry& registry_;
  std::vector<LoadedPlugin> plugins_;
};

}  // namespace robot_runtime
