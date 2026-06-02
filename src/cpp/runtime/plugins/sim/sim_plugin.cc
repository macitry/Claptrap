#include "plugins/sim/sim_plugin.hpp"

#include <iostream>
#include <memory>
#include <string>

namespace robot_runtime {
namespace {

std::string Option(
    const PluginSpec& spec,
    const std::string& key,
    const std::string& fallback) {
  const auto it = spec.config.find(key);
  return it == spec.config.end() ? fallback : it->second;
}

class SimPlugin final : public Plugin {
 public:
  void Configure(const PluginSpec& spec, const PluginContext&) override {
    name_ = spec.name;
    scene_ = Option(spec, "scene", "");
    shared_memory_config_ = Option(spec, "shared_memory_config", "");
  }

  void Start() override {
    std::cout << "[sim] scene=" << scene_
              << " shared_memory_config=" << shared_memory_config_ << "\n";
  }

  void Stop() override {
    std::cout << "[sim] stopped " << name_ << "\n";
  }

 private:
  std::string name_;
  std::string scene_;
  std::string shared_memory_config_;
};

}  // namespace

void RegisterSimPlugin(PluginRegistry& registry) {
  registry.Register("sim", [] {
    return std::make_unique<SimPlugin>();
  });
}

}  // namespace robot_runtime
