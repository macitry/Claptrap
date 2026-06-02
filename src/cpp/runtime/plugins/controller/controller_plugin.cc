#include "plugins/controller/controller_plugin.hpp"

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

class ControllerPlugin final : public Plugin {
 public:
  void Configure(const PluginSpec& spec, const PluginContext&) override {
    name_ = spec.name;
    command_target_ = Option(spec, "command_target", "");
  }

  void Start() override {
    std::cout << "[controller] command_target=" << command_target_ << "\n";
  }

  void Stop() override {
    std::cout << "[controller] stopped " << name_ << "\n";
  }

 private:
  std::string name_;
  std::string command_target_;
};

}  // namespace

void RegisterControllerPlugin(PluginRegistry& registry) {
  registry.Register("controller", [] {
    return std::make_unique<ControllerPlugin>();
  });
}

}  // namespace robot_runtime
