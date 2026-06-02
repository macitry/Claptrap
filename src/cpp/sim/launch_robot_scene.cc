#include "robot_shared_memory.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

namespace {

constexpr double kDefaultDurationSeconds = 50.0;

struct Args {
  std::string scene;
  double duration = kDefaultDurationSeconds;
  std::string shm_name;
  std::string shm_config = robot_sim::kDefaultConfigPath;
  bool no_unlink = false;
};

void PrintUsage(const char* program) {
  std::cerr
      << "Usage: " << program << " --scene SCENE [--duration SECONDS]\n"
      << "       [--shm-name NAME] [--shm-config CONFIG] [--no-unlink]\n";
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

    if (option == "--scene") {
      args.scene = require_value(option);
    } else if (option == "--duration") {
      args.duration = std::stod(require_value(option));
    } else if (option == "--shm-name") {
      args.shm_name = require_value(option);
    } else if (option == "--shm-config") {
      args.shm_config = require_value(option);
    } else if (option == "--no-unlink") {
      args.no_unlink = true;
    } else if (option == "-h" || option == "--help") {
      PrintUsage(argv[0]);
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + option);
    }
  }

  if (args.scene.empty()) {
    throw std::runtime_error("--scene is required");
  }
  if (args.duration <= 0.0) {
    throw std::runtime_error("--duration must be greater than 0");
  }
  return args;
}

std::vector<double> CopyArray(const mjtNum* values, int count) {
  std::vector<double> result;
  if (count <= 0) {
    return result;
  }
  result.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    result.push_back(static_cast<double>(values[i]));
  }
  return result;
}

std::vector<double> StateFieldValue(
    const std::string& name,
    const mjModel* model,
    const mjData* data) {
  if (name == "time") {
    return {data->time};
  }
  if (name == "qpos") {
    return CopyArray(data->qpos, model->nq);
  }
  if (name == "qvel") {
    return CopyArray(data->qvel, model->nv);
  }
  if (name == "qacc") {
    return CopyArray(data->qacc, model->nv);
  }
  if (name == "qacc_warmstart") {
    return CopyArray(data->qacc_warmstart, model->nv);
  }
  if (name == "sensordata") {
    return CopyArray(data->sensordata, model->nsensordata);
  }
  if (name == "ctrl") {
    return CopyArray(data->ctrl, model->nu);
  }
  if (name == "actuator_force") {
    return CopyArray(data->actuator_force, model->nu);
  }
  if (name == "qfrc_applied") {
    return CopyArray(data->qfrc_applied, model->nv);
  }
  if (name == "qfrc_actuator") {
    return CopyArray(data->qfrc_actuator, model->nv);
  }
  if (name == "qfrc_smooth") {
    return CopyArray(data->qfrc_smooth, model->nv);
  }
  if (name == "qfrc_constraint") {
    return CopyArray(data->qfrc_constraint, model->nv);
  }
  if (name == "qfrc_inverse") {
    return CopyArray(data->qfrc_inverse, model->nv);
  }
  if (name == "xfrc_applied") {
    return CopyArray(data->xfrc_applied, 6 * model->nbody);
  }
  throw std::runtime_error("no MuJoCo data source for state field: " + name);
}

std::map<std::string, std::vector<double>> StateFields(
    const robot_sim::RobotSharedMemory& shared_io,
    const mjModel* model,
    const mjData* data) {
  std::map<std::string, std::vector<double>> fields;
  for (const robot_sim::FieldLayout& field : shared_io.layout().state_fields) {
    fields[field.name] = StateFieldValue(field.name, model, data);
  }
  return fields;
}

void WriteSimState(
    robot_sim::RobotSharedMemory& shared_io,
    const mjModel* model,
    const mjData* data) {
  shared_io.WriteState(
      data->time,
      model->opt.timestep,
      StateFields(shared_io, model, data));
}

std::vector<double> ClipTorqueToCtrlRange(
    const mjModel* model,
    const std::vector<double>& torque) {
  std::vector<double> result = torque;
  for (int i = 0; i < model->nu && i < static_cast<int>(result.size()); ++i) {
    if (model->actuator_ctrllimited[i]) {
      const double low = model->actuator_ctrlrange[2 * i];
      const double high = model->actuator_ctrlrange[2 * i + 1];
      result[i] = std::min(std::max(result[i], low), high);
    }
  }
  return result;
}

void ApplyExternalCommand(
    robot_sim::RobotSharedMemory& shared_io,
    const mjModel* model,
    mjData* data) {
  robot_sim::RobotCommand command;
  try {
    command = shared_io.ReadCommand(std::chrono::duration<double>(0.001));
  } catch (const std::exception&) {
    return;
  }

  if (command.enabled && command.mode == robot_sim::kCommandModeTorque) {
    const auto it = command.fields.find("torque");
    if (it == command.fields.end()) {
      throw std::runtime_error("enabled torque command has no torque field");
    }
    const std::vector<double> torque = ClipTorqueToCtrlRange(model, it->second);
    for (int i = 0; i < model->nu; ++i) {
      data->ctrl[i] = i < static_cast<int>(torque.size()) ? torque[i] : 0.0;
    }
  } else {
    for (int i = 0; i < model->nu; ++i) {
      data->ctrl[i] = 0.0;
    }
  }
}

std::string ActuatorName(const mjModel* model, int actuator_id) {
  const char* name = mj_id2name(model, mjOBJ_ACTUATOR, actuator_id);
  if (name && name[0] != '\0') {
    return name;
  }
  return "actuator_" + std::to_string(actuator_id);
}

std::string JointName(const mjModel* model, int joint_id) {
  const char* name = mj_id2name(model, mjOBJ_JOINT, joint_id);
  if (name && name[0] != '\0') {
    return name;
  }
  return "joint_" + std::to_string(joint_id);
}

void PrintNames(const mjModel* model) {
  std::cout << "Joints: ";
  for (int i = 0; i < model->njnt; ++i) {
    if (i != 0) {
      std::cout << ", ";
    }
    std::cout << JointName(model, i);
  }
  std::cout << "\nTorque command order: ";
  for (int i = 0; i < model->nu; ++i) {
    if (i != 0) {
      std::cout << ", ";
    }
    std::cout << ActuatorName(model, i);
  }
  std::cout << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Args args = ParseArgs(argc, argv);

    char error[1024] = {};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
        mj_loadXML(args.scene.c_str(), nullptr, error, sizeof(error)),
        mj_deleteModel);
    if (!model) {
      throw std::runtime_error(std::string("failed to load scene: ") + error);
    }
    std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
        mj_makeData(model.get()),
        mj_deleteData);
    if (!data) {
      throw std::runtime_error("failed to allocate MuJoCo data");
    }

    const robot_sim::ModelDimensions dimensions{
        static_cast<std::uint32_t>(model->nq),
        static_cast<std::uint32_t>(model->nv),
        static_cast<std::uint32_t>(model->nu),
        static_cast<std::uint32_t>(model->nsensordata),
    };
    robot_sim::RobotSharedMemory shared_io = robot_sim::RobotSharedMemory::Create(
        dimensions,
        model->opt.timestep,
        args.shm_config,
        args.shm_name,
        true,
        !args.no_unlink);

    std::cout << "Loading scene: " << args.scene << "\n";
    std::cout << "C++ headless simulation will run for " << args.duration
              << " seconds.\n";
    std::cout << "Shared memory: " << shared_io.name() << "\n";
    PrintNames(model.get());

    WriteSimState(shared_io, model.get(), data.get());

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration<double>(args.duration);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto step_start = std::chrono::steady_clock::now();
      ApplyExternalCommand(shared_io, model.get(), data.get());
      mj_step(model.get(), data.get());
      WriteSimState(shared_io, model.get(), data.get());

      const auto elapsed = std::chrono::steady_clock::now() - step_start;
      const auto timestep = std::chrono::duration<double>(model->opt.timestep);
      if (elapsed < timestep) {
        std::this_thread::sleep_for(timestep - elapsed);
      }
    }
    return 0;
  } catch (const std::exception& exc) {
    std::cerr << "error: " << exc.what() << "\n";
    PrintUsage(argv[0]);
    return 1;
  }
}
