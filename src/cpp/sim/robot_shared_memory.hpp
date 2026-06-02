#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace robot_sim {

constexpr char kMagic[8] = {'R', 'O', 'B', 'O', 'T', 'I', 'O', '\0'};
constexpr std::uint32_t kVersion = 2;
constexpr std::uint32_t kCommandModeTorque = 0;
constexpr const char* kDefaultConfigPath =
    "src/config/robot_shared_memory_config.json";
constexpr const char* kDefaultSharedMemoryName = "robot_mujoco_io";

struct ModelDimensions {
  std::uint32_t nq = 0;
  std::uint32_t nv = 0;
  std::uint32_t nu = 0;
  std::uint32_t nsensordata = 0;
};

struct FieldSpec {
  std::string name;
  std::string dtype;
  bool size_is_dimension = false;
  std::string size_dimension;
  std::uint32_t fixed_size = 0;
};

struct SharedMemoryConfig {
  std::string shared_memory_name = kDefaultSharedMemoryName;
  std::vector<FieldSpec> state_fields;
  std::vector<FieldSpec> command_fields;
  std::uint64_t fingerprint = 0;

  static SharedMemoryConfig Load(const std::string& path);
};

struct FieldLayout {
  std::string name;
  std::size_t offset = 0;
  std::uint32_t size = 0;
};

struct SharedMemoryLayout {
  ModelDimensions dimensions;
  std::vector<FieldLayout> state_fields;
  std::vector<FieldLayout> command_fields;
  std::map<std::string, std::size_t> state_index;
  std::map<std::string, std::size_t> command_index;
  std::size_t total_size = 0;

  static SharedMemoryLayout FromConfig(
      const SharedMemoryConfig& config,
      const ModelDimensions& dimensions);
};

struct SharedMemoryHeader {
  std::uint64_t total_size = 0;
  std::uint64_t config_fingerprint = 0;
  ModelDimensions dimensions;
  std::uint64_t state_seq = 0;
  std::uint64_t command_seq = 0;
  bool sim_alive = false;
  bool command_enabled = false;
  std::uint32_t command_mode = 0;
  double sim_time = 0.0;
  double timestep = 0.0;
  double state_wall_time = 0.0;
  double command_wall_time = 0.0;
};

struct RobotState {
  std::uint64_t sequence = 0;
  bool sim_alive = false;
  double sim_time = 0.0;
  double timestep = 0.0;
  double wall_time = 0.0;
  std::map<std::string, std::vector<double>> fields;
};

struct RobotCommand {
  std::uint64_t sequence = 0;
  bool enabled = false;
  std::uint32_t mode = 0;
  double wall_time = 0.0;
  std::map<std::string, std::vector<double>> fields;
};

class RobotSharedMemory {
 public:
  RobotSharedMemory() = default;
  RobotSharedMemory(const RobotSharedMemory&) = delete;
  RobotSharedMemory& operator=(const RobotSharedMemory&) = delete;
  RobotSharedMemory(RobotSharedMemory&& other) noexcept;
  RobotSharedMemory& operator=(RobotSharedMemory&& other) noexcept;
  ~RobotSharedMemory();

  static RobotSharedMemory Create(
      const ModelDimensions& dimensions,
      double timestep,
      const std::string& config_path = kDefaultConfigPath,
      const std::string& name_override = "",
      bool unlink_existing = true,
      bool unlink_on_close = true);

  static RobotSharedMemory Attach(
      const std::string& config_path = kDefaultConfigPath,
      const std::string& name_override = "");

  const std::string& name() const { return name_; }
  const SharedMemoryLayout& layout() const { return layout_; }
  const SharedMemoryConfig& config() const { return config_; }

  SharedMemoryHeader ReadHeader() const;
  void SetAlive(bool alive);
  void WriteState(
      double sim_time,
      double timestep,
      const std::map<std::string, std::vector<double>>& fields);
  RobotState ReadState(std::chrono::duration<double> timeout) const;
  void WriteCommand(
      bool enabled,
      std::uint32_t mode,
      const std::map<std::string, std::vector<double>>& fields);
  void WriteTorque(const std::vector<double>& torque, bool enabled = true);
  void DisableCommand();
  RobotCommand ReadCommand(std::chrono::duration<double> timeout) const;
  void Close();

 private:
  int fd_ = -1;
  std::uint8_t* memory_ = nullptr;
  std::size_t mapped_size_ = 0;
  bool owner_ = false;
  bool unlink_on_close_ = false;
  std::string name_;
  std::string posix_name_;
  SharedMemoryConfig config_;
  SharedMemoryLayout layout_;

  void WriteCommonHeader() const;
  void WriteStateHeader(
      std::uint64_t sequence,
      bool sim_alive,
      double sim_time,
      double timestep,
      double wall_time) const;
  void WriteCommandHeader(
      std::uint64_t sequence,
      bool enabled,
      std::uint32_t mode,
      double wall_time) const;
  void ZeroArrays() const;
  void ValidateFields(
      const std::map<std::string, std::vector<double>>& fields,
      const std::vector<FieldLayout>& layouts,
      const std::map<std::string, std::size_t>& index,
      const std::string& label) const;
  void WriteFields(
      const std::map<std::string, std::vector<double>>& fields,
      const std::vector<FieldLayout>& layouts) const;
  std::map<std::string, std::vector<double>> ReadFields(
      const std::vector<FieldLayout>& layouts) const;
};

double WallTimeSeconds();

}  // namespace robot_sim
