#pragma once

#include <string>
#include <vector>

#include "core/plugin.hpp"

namespace robot_runtime {

struct RuntimeConfig {
  std::vector<PluginSpec> plugins;

  static RuntimeConfig Load(const std::string& path);
};

}  // namespace robot_runtime
