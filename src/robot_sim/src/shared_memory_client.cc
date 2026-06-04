#include "robot_shared_memory.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Args {
  std::string name;
  std::string config = robot_sim::kDefaultConfigPath;
  bool has_torque = false;
  std::vector<double> torque;
  bool disable_command = false;
  double duration = 0.0;
  double rate = 50.0;
};

void PrintUsage(const char* program) {
  std::cerr
      << "Usage: " << program << " [--name NAME] [--config CONFIG]\n"
      << "       [--torque VALUES...] [--disable-command]\n"
      << "       [--duration SECONDS] [--rate HZ]\n";
}

bool IsOption(const std::string& value) {
  return value.rfind("--", 0) == 0;
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

    if (option == "--name") {
      args.name = require_value(option);
    } else if (option == "--config") {
      args.config = require_value(option);
    } else if (option == "--torque") {
      args.has_torque = true;
      while (i + 1 < argc && !IsOption(argv[i + 1])) {
        args.torque.push_back(std::stod(argv[++i]));
      }
    } else if (option == "--disable-command") {
      args.disable_command = true;
    } else if (option == "--duration") {
      args.duration = std::stod(require_value(option));
    } else if (option == "--rate") {
      args.rate = std::stod(require_value(option));
    } else if (option == "-h" || option == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + option);
    }
  }
  if (args.rate <= 0.0) {
    throw std::runtime_error("--rate must be greater than 0");
  }
  return args;
}

std::string FormatValues(const std::vector<double>& values, std::size_t limit = 6) {
  std::ostringstream output;
  output << "[";
  for (std::size_t i = 0; i < values.size() && i < limit; ++i) {
    if (i != 0) {
      output << ", ";
    }
    output << values[i];
  }
  if (values.size() > limit) {
    output << ", ...";
  }
  output << "]";
  return output.str();
}

std::string RunCycle(robot_sim::RobotSharedMemory& shared_io, const Args& args) {
  robot_sim::RobotState state =
      shared_io.ReadState(std::chrono::duration<double>(1.0));

  if (args.disable_command) {
    shared_io.DisableCommand();
  } else if (args.has_torque) {
    shared_io.WriteTorque(args.torque);
  }

  std::ostringstream output;
  output << "t=" << state.sim_time
         << " alive=" << (state.sim_alive ? 1 : 0);
  for (const auto& field : state.fields) {
    output << " " << field.first << "=" << FormatValues(field.second);
  }
  return output.str();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = ParseArgs(argc, argv);
    robot_sim::RobotSharedMemory shared_io =
        robot_sim::RobotSharedMemory::Attach(args.config, args.name);

    if (args.has_torque) {
      const auto it = shared_io.layout().command_index.find("torque");
      if (it == shared_io.layout().command_index.end()) {
        throw std::runtime_error("the JSON config does not define torque");
      }
      const robot_sim::FieldLayout& torque_field =
          shared_io.layout().command_fields[it->second];
      if (args.torque.size() != torque_field.size) {
        throw std::runtime_error(
            "--torque needs " + std::to_string(torque_field.size) +
            " values for this config, got " +
            std::to_string(args.torque.size()));
      }
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(args.duration);
    while (true) {
      std::cout << RunCycle(shared_io, args) << "\n";
      if (args.duration <= 0.0 || std::chrono::steady_clock::now() >= deadline) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / args.rate));
    }
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "error: " << exc.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
