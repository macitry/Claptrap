#pragma once

#include <map>
#include <string>

namespace robot_runtime {

struct PluginSpec {
  std::string name;
  std::string type;
  bool enabled = true;
  std::map<std::string, std::string> config;
};

struct PluginContext {
  std::string config_path;
};

class Plugin {
 public:
  virtual ~Plugin() = default;

  virtual void Configure(const PluginSpec& spec, const PluginContext& context) = 0;
  virtual void Start() = 0;
  virtual void Stop() = 0;
};

}  // namespace robot_runtime
