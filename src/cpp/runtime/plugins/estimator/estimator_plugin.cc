#include "plugins/estimator/estimator_plugin.hpp"

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

class EstimatorPlugin final : public Plugin {
 public:
  void Configure(const PluginSpec& spec, const PluginContext&) override {
    name_ = spec.name;
    source_ = Option(spec, "source", "");
  }

  void Start() override {
    std::cout << "[estimator] source=" << source_ << "\n";
  }

  void Stop() override {
    std::cout << "[estimator] stopped " << name_ << "\n";
  }

 private:
  std::string name_;
  std::string source_;
};

}  // namespace

void RegisterEstimatorPlugin(PluginRegistry& registry) {
  registry.Register("estimator", [] {
    return std::make_unique<EstimatorPlugin>();
  });
}

}  // namespace robot_runtime
