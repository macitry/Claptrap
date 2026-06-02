#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "core/plugin.hpp"

namespace robot_runtime {

class PluginRegistry {
 public:
  using Factory = std::function<std::unique_ptr<Plugin>()>;

  void Register(const std::string& type, Factory factory);
  std::unique_ptr<Plugin> Create(const std::string& type) const;
  std::vector<std::string> RegisteredTypes() const;

 private:
  std::map<std::string, Factory> factories_;
};

}  // namespace robot_runtime
