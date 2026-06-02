#include "core/plugin_manager.hpp"

#include <iostream>
#include <stdexcept>

namespace robot_runtime {

PluginManager::PluginManager(const PluginRegistry& registry) : registry_(registry) {}

void PluginManager::Load(
    const RuntimeConfig& config,
    const PluginContext& context) {
  plugins_.clear();
  for (const PluginSpec& spec : config.plugins) {
    if (!spec.enabled) {
      std::cout << "[runtime] skip disabled plugin: " << spec.name << "\n";
      continue;
    }

    LoadedPlugin loaded;
    loaded.spec = spec;
    loaded.plugin = registry_.Create(spec.type);
    std::cout << "[runtime] configure " << spec.name
              << " type=" << spec.type << "\n";
    loaded.plugin->Configure(spec, context);
    plugins_.push_back(std::move(loaded));
  }
}

void PluginManager::Start() {
  try {
    for (LoadedPlugin& loaded : plugins_) {
      std::cout << "[runtime] start " << loaded.spec.name << "\n";
      loaded.plugin->Start();
      loaded.started = true;
    }
  } catch (...) {
    Stop();
    throw;
  }
}

void PluginManager::Stop() {
  for (auto it = plugins_.rbegin(); it != plugins_.rend(); ++it) {
    if (!it->started) {
      continue;
    }
    std::cout << "[runtime] stop " << it->spec.name << "\n";
    try {
      it->plugin->Stop();
    } catch (const std::exception& exc) {
      std::cerr << "[runtime] stop failed for " << it->spec.name
                << ": " << exc.what() << "\n";
    }
    it->started = false;
  }
}

}  // namespace robot_runtime
