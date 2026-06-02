#include "core/plugin_registry.hpp"

#include <stdexcept>

namespace robot_runtime {

void PluginRegistry::Register(const std::string& type, Factory factory) {
  if (type.empty()) {
    throw std::runtime_error("plugin type must not be empty");
  }
  if (!factory) {
    throw std::runtime_error("plugin factory must be valid for type: " + type);
  }
  if (!factories_.emplace(type, std::move(factory)).second) {
    throw std::runtime_error("duplicate plugin type: " + type);
  }
}

std::unique_ptr<Plugin> PluginRegistry::Create(const std::string& type) const {
  const auto it = factories_.find(type);
  if (it == factories_.end()) {
    throw std::runtime_error("unknown plugin type: " + type);
  }
  return it->second();
}

std::vector<std::string> PluginRegistry::RegisteredTypes() const {
  std::vector<std::string> types;
  types.reserve(factories_.size());
  for (const auto& entry : factories_) {
    types.push_back(entry.first);
  }
  return types;
}

}  // namespace robot_runtime
