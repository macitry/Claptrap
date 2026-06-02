#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "core/plugin_manager.hpp"
#include "core/plugin_registry.hpp"
#include "core/runtime_config.hpp"
#include "plugins/controller/controller_plugin.hpp"
#include "plugins/estimator/estimator_plugin.hpp"
#include "plugins/sim/sim_plugin.hpp"

namespace {

constexpr const char* kDefaultRuntimeConfig = "src/config/robot_runtime_config.json";

std::atomic<bool> g_stop_requested{false};

struct Args {
  std::string config = kDefaultRuntimeConfig;
  double duration = -1.0;
};

void RequestStop(int) {
  g_stop_requested.store(true);
}

void PrintUsage(const char* program) {
  std::cerr << "Usage: " << program
            << " [--config CONFIG] [--duration SECONDS]\n";
}

Args ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    auto require_value = [&](const std::string& name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(name + " requires a value");
      }
      return argv[++i];
    };

    if (option == "--config") {
      args.config = require_value(option);
    } else if (option == "--duration") {
      args.duration = std::stod(require_value(option));
    } else if (option == "-h" || option == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + option);
    }
  }
  return args;
}

void WaitForStop(double duration) {
  using namespace std::chrono_literals;
  if (duration >= 0.0) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);
    while (!g_stop_requested.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(50ms);
    }
    return;
  }

  while (!g_stop_requested.load()) {
    std::this_thread::sleep_for(100ms);
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::signal(SIGINT, RequestStop);
    std::signal(SIGTERM, RequestStop);

    const Args args = ParseArgs(argc, argv);
    robot_runtime::PluginRegistry registry;
    robot_runtime::RegisterSimPlugin(registry);
    robot_runtime::RegisterEstimatorPlugin(registry);
    robot_runtime::RegisterControllerPlugin(registry);

    const robot_runtime::RuntimeConfig config =
        robot_runtime::RuntimeConfig::Load(args.config);
    robot_runtime::PluginContext context;
    context.config_path = args.config;

    robot_runtime::PluginManager manager(registry);
    manager.Load(config, context);
    manager.Start();
    WaitForStop(args.duration);
    manager.Stop();
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "error: " << exc.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
