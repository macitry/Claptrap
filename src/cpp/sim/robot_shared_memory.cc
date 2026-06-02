#include "robot_shared_memory.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <openssl/sha.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace robot_sim {
namespace {

constexpr std::size_t kDoubleSize = 8;

#pragma pack(push, 1)
struct CommonHeaderRaw {
  char magic[8];
  std::uint32_t version;
  std::uint32_t arrays_offset;
  std::uint64_t total_size;
  std::uint32_t nq;
  std::uint32_t nv;
  std::uint32_t nu;
  std::uint32_t nsensordata;
  std::uint64_t config_fingerprint;
};

struct StateHeaderRaw {
  std::uint64_t sequence;
  std::uint32_t sim_alive;
  std::uint32_t reserved;
  double sim_time;
  double timestep;
  double wall_time;
};

struct CommandHeaderRaw {
  std::uint64_t sequence;
  std::uint32_t enabled;
  std::uint32_t mode;
  double wall_time;
};
#pragma pack(pop)

static_assert(sizeof(double) == kDoubleSize, "shared memory requires 64-bit double");
static_assert(sizeof(CommonHeaderRaw) == 48, "common header ABI mismatch");
static_assert(sizeof(StateHeaderRaw) == 40, "state header ABI mismatch");
static_assert(sizeof(CommandHeaderRaw) == 24, "command header ABI mismatch");

constexpr std::size_t kCommonHeaderOffset = 0;
constexpr std::size_t kStateHeaderOffset =
    kCommonHeaderOffset + sizeof(CommonHeaderRaw);
constexpr std::size_t kCommandHeaderOffset =
    kStateHeaderOffset + sizeof(StateHeaderRaw);
constexpr std::size_t kArraysOffset =
    kCommandHeaderOffset + sizeof(CommandHeaderRaw);

std::string ReadTextFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open file: " + path);
  }
  return std::string(
      std::istreambuf_iterator<char>(file),
      std::istreambuf_iterator<char>());
}

[[noreturn]] void ThrowErrno(const std::string& message) {
  throw std::runtime_error(message + ": " + std::strerror(errno));
}

std::string MakePosixName(const std::string& name) {
  if (name.empty()) {
    throw std::runtime_error("shared-memory name must not be empty");
  }
  if (name.front() == '/') {
    return name;
  }
  return "/" + name;
}

std::uint64_t NextOddSequence(std::uint64_t sequence) {
  std::uint64_t odd_sequence = sequence + 1;
  if (odd_sequence % 2 == 0) {
    ++odd_sequence;
  }
  return odd_sequence;
}

struct JsonValue {
  enum class Type {
    kNull,
    kObject,
    kArray,
    kString,
    kInteger,
  };

  Type type = Type::kNull;
  std::map<std::string, JsonValue> object;
  std::vector<JsonValue> array;
  std::string string;
  std::int64_t integer = 0;
};

class JsonParser {
 public:
  explicit JsonParser(std::string text) : text_(std::move(text)) {}

  JsonValue Parse() {
    JsonValue value = ParseValue();
    SkipWhitespace();
    if (position_ != text_.size()) {
      throw std::runtime_error("unexpected trailing JSON content");
    }
    return value;
  }

 private:
  JsonValue ParseValue() {
    SkipWhitespace();
    if (position_ >= text_.size()) {
      throw std::runtime_error("unexpected end of JSON");
    }

    const char ch = text_[position_];
    if (ch == '{') {
      return ParseObject();
    }
    if (ch == '[') {
      return ParseArray();
    }
    if (ch == '"') {
      JsonValue value;
      value.type = JsonValue::Type::kString;
      value.string = ParseString();
      return value;
    }
    if (ch == '-' || std::isdigit(static_cast<unsigned char>(ch))) {
      return ParseInteger();
    }
    if (text_.compare(position_, 4, "null") == 0) {
      position_ += 4;
      return JsonValue{};
    }
    throw std::runtime_error("unsupported JSON value");
  }

  JsonValue ParseObject() {
    Expect('{');
    JsonValue value;
    value.type = JsonValue::Type::kObject;
    SkipWhitespace();
    if (Consume('}')) {
      return value;
    }

    while (true) {
      SkipWhitespace();
      std::string key = ParseString();
      SkipWhitespace();
      Expect(':');
      value.object.emplace(std::move(key), ParseValue());
      SkipWhitespace();
      if (Consume('}')) {
        break;
      }
      Expect(',');
    }
    return value;
  }

  JsonValue ParseArray() {
    Expect('[');
    JsonValue value;
    value.type = JsonValue::Type::kArray;
    SkipWhitespace();
    if (Consume(']')) {
      return value;
    }

    while (true) {
      value.array.push_back(ParseValue());
      SkipWhitespace();
      if (Consume(']')) {
        break;
      }
      Expect(',');
    }
    return value;
  }

  std::string ParseString() {
    Expect('"');
    std::string result;
    while (position_ < text_.size()) {
      const char ch = text_[position_++];
      if (ch == '"') {
        return result;
      }
      if (ch != '\\') {
        result.push_back(ch);
        continue;
      }
      if (position_ >= text_.size()) {
        throw std::runtime_error("unterminated JSON escape");
      }
      const char escaped = text_[position_++];
      switch (escaped) {
        case '"':
        case '\\':
        case '/':
          result.push_back(escaped);
          break;
        case 'b':
          result.push_back('\b');
          break;
        case 'f':
          result.push_back('\f');
          break;
        case 'n':
          result.push_back('\n');
          break;
        case 'r':
          result.push_back('\r');
          break;
        case 't':
          result.push_back('\t');
          break;
        default:
          throw std::runtime_error("unsupported JSON escape");
      }
    }
    throw std::runtime_error("unterminated JSON string");
  }

  JsonValue ParseInteger() {
    const std::size_t start = position_;
    if (text_[position_] == '-') {
      ++position_;
    }
    if (position_ >= text_.size() ||
        !std::isdigit(static_cast<unsigned char>(text_[position_]))) {
      throw std::runtime_error("invalid JSON integer");
    }
    while (position_ < text_.size() &&
           std::isdigit(static_cast<unsigned char>(text_[position_]))) {
      ++position_;
    }
    if (position_ < text_.size() && text_[position_] == '.') {
      throw std::runtime_error("floating-point JSON numbers are not supported here");
    }

    JsonValue value;
    value.type = JsonValue::Type::kInteger;
    value.integer = std::stoll(text_.substr(start, position_ - start));
    return value;
  }

  void SkipWhitespace() {
    while (position_ < text_.size() &&
           std::isspace(static_cast<unsigned char>(text_[position_]))) {
      ++position_;
    }
  }

  void Expect(char expected) {
    SkipWhitespace();
    if (position_ >= text_.size() || text_[position_] != expected) {
      throw std::runtime_error(std::string("expected JSON character: ") + expected);
    }
    ++position_;
  }

  bool Consume(char expected) {
    SkipWhitespace();
    if (position_ < text_.size() && text_[position_] == expected) {
      ++position_;
      return true;
    }
    return false;
  }

  std::string text_;
  std::size_t position_ = 0;
};

const JsonValue& RequireObject(const JsonValue& value, const std::string& label) {
  if (value.type != JsonValue::Type::kObject) {
    throw std::runtime_error(label + " must be a JSON object");
  }
  return value;
}

const JsonValue* FindKey(const JsonValue& object, const std::string& key) {
  RequireObject(object, "config");
  auto it = object.object.find(key);
  if (it == object.object.end()) {
    return nullptr;
  }
  return &it->second;
}

std::string OptionalString(
    const JsonValue& object,
    const std::string& key,
    const std::string& fallback) {
  const JsonValue* value = FindKey(object, key);
  if (!value) {
    return fallback;
  }
  if (value->type != JsonValue::Type::kString || value->string.empty()) {
    throw std::runtime_error(key + " must be a non-empty string");
  }
  return value->string;
}

std::string RequiredString(const JsonValue& object, const std::string& key) {
  const JsonValue* value = FindKey(object, key);
  if (!value || value->type != JsonValue::Type::kString || value->string.empty()) {
    throw std::runtime_error(key + " must be a non-empty string");
  }
  return value->string;
}

std::vector<FieldSpec> LoadFieldSpecs(
    const JsonValue& object,
    const std::string& key) {
  const JsonValue* raw_fields = FindKey(object, key);
  if (!raw_fields || raw_fields->type != JsonValue::Type::kArray) {
    throw std::runtime_error(key + " must be a JSON array");
  }

  std::vector<FieldSpec> fields;
  fields.reserve(raw_fields->array.size());
  for (const JsonValue& raw_field : raw_fields->array) {
    RequireObject(raw_field, "field");
    FieldSpec field;
    field.name = RequiredString(raw_field, "name");
    field.dtype = OptionalString(raw_field, "dtype", "float64");
    if (field.dtype != "float64") {
      throw std::runtime_error(
          "unsupported dtype for field " + field.name + ": " + field.dtype);
    }

    const JsonValue* size = FindKey(raw_field, "size");
    if (!size) {
      throw std::runtime_error("field " + field.name + " requires size");
    }
    if (size->type == JsonValue::Type::kString) {
      field.size_is_dimension = true;
      field.size_dimension = size->string;
    } else if (size->type == JsonValue::Type::kInteger) {
      if (size->integer < 0 ||
          size->integer > std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error("invalid size for field " + field.name);
      }
      field.fixed_size = static_cast<std::uint32_t>(size->integer);
    } else {
      throw std::runtime_error(
          "field " + field.name + " size must be an integer or dimension key");
    }
    fields.push_back(std::move(field));
  }
  return fields;
}

void ValidateUniqueNames(
    const std::vector<FieldSpec>& fields,
    const std::string& label) {
  std::set<std::string> names;
  for (const FieldSpec& field : fields) {
    if (!names.insert(field.name).second) {
      throw std::runtime_error(label + " has duplicate field name: " + field.name);
    }
  }
}

std::uint32_t ResolveSize(
    const FieldSpec& field,
    const ModelDimensions& dimensions) {
  if (!field.size_is_dimension) {
    return field.fixed_size;
  }
  if (field.size_dimension == "nq") {
    return dimensions.nq;
  }
  if (field.size_dimension == "nv") {
    return dimensions.nv;
  }
  if (field.size_dimension == "nu") {
    return dimensions.nu;
  }
  if (field.size_dimension == "nsensordata") {
    return dimensions.nsensordata;
  }
  throw std::runtime_error(
      "field " + field.name + " references unknown dimension " +
      field.size_dimension);
}

std::string QuoteJsonString(const std::string& value) {
  std::string result = "\"";
  for (char ch : value) {
    switch (ch) {
      case '"':
        result += "\\\"";
        break;
      case '\\':
        result += "\\\\";
        break;
      case '\b':
        result += "\\b";
        break;
      case '\f':
        result += "\\f";
        break;
      case '\n':
        result += "\\n";
        break;
      case '\r':
        result += "\\r";
        break;
      case '\t':
        result += "\\t";
        break;
      default:
        result.push_back(ch);
        break;
    }
  }
  result += "\"";
  return result;
}

std::string FieldSpecNormalizedJson(const FieldSpec& field) {
  std::string result = "{\"dtype\":";
  result += QuoteJsonString(field.dtype);
  result += ",\"name\":";
  result += QuoteJsonString(field.name);
  result += ",\"size\":";
  if (field.size_is_dimension) {
    result += QuoteJsonString(field.size_dimension);
  } else {
    result += std::to_string(field.fixed_size);
  }
  result += "}";
  return result;
}

std::string FieldArrayNormalizedJson(const std::vector<FieldSpec>& fields) {
  std::string result = "[";
  for (std::size_t i = 0; i < fields.size(); ++i) {
    if (i != 0) {
      result += ",";
    }
    result += FieldSpecNormalizedJson(fields[i]);
  }
  result += "]";
  return result;
}

std::string ConfigNormalizedJson(const SharedMemoryConfig& config) {
  std::string result = "{\"command_fields\":";
  result += FieldArrayNormalizedJson(config.command_fields);
  result += ",\"shared_memory_name\":";
  result += QuoteJsonString(config.shared_memory_name);
  result += ",\"state_fields\":";
  result += FieldArrayNormalizedJson(config.state_fields);
  result += "}";
  return result;
}

std::uint64_t FingerprintConfig(const SharedMemoryConfig& config) {
  const std::string normalized = ConfigNormalizedJson(config);
  unsigned char digest[SHA256_DIGEST_LENGTH] = {};
  SHA256(
      reinterpret_cast<const unsigned char*>(normalized.data()),
      normalized.size(),
      digest);

  std::uint64_t fingerprint = 0;
  for (int i = 0; i < 8; ++i) {
    fingerprint |= static_cast<std::uint64_t>(digest[i]) << (8 * i);
  }
  return fingerprint;
}

template <typename T>
T ReadStruct(const std::uint8_t* memory, std::size_t offset) {
  T value;
  std::memcpy(&value, memory + offset, sizeof(T));
  return value;
}

template <typename T>
void WriteStruct(std::uint8_t* memory, std::size_t offset, const T& value) {
  std::memcpy(memory + offset, &value, sizeof(T));
}

CommonHeaderRaw ReadCommonHeaderRaw(const std::uint8_t* memory) {
  CommonHeaderRaw header =
      ReadStruct<CommonHeaderRaw>(memory, kCommonHeaderOffset);
  if (std::memcmp(header.magic, kMagic, sizeof(header.magic)) != 0) {
    throw std::runtime_error("invalid shared-memory magic");
  }
  if (header.version != kVersion) {
    throw std::runtime_error("unsupported shared-memory version");
  }
  if (header.arrays_offset != kArraysOffset) {
    throw std::runtime_error("unsupported shared-memory arrays offset");
  }
  return header;
}

StateHeaderRaw ReadStateHeaderRaw(const std::uint8_t* memory) {
  return ReadStruct<StateHeaderRaw>(memory, kStateHeaderOffset);
}

CommandHeaderRaw ReadCommandHeaderRaw(const std::uint8_t* memory) {
  return ReadStruct<CommandHeaderRaw>(memory, kCommandHeaderOffset);
}

}  // namespace

double WallTimeSeconds() {
  using Clock = std::chrono::system_clock;
  const auto now = Clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

SharedMemoryConfig SharedMemoryConfig::Load(const std::string& path) {
  JsonParser parser(ReadTextFile(path));
  const JsonValue raw = parser.Parse();
  RequireObject(raw, "config");

  SharedMemoryConfig config;
  config.shared_memory_name =
      OptionalString(raw, "shared_memory_name", kDefaultSharedMemoryName);
  config.state_fields = LoadFieldSpecs(raw, "state_fields");
  config.command_fields = LoadFieldSpecs(raw, "command_fields");
  ValidateUniqueNames(config.state_fields, "state_fields");
  ValidateUniqueNames(config.command_fields, "command_fields");
  config.fingerprint = FingerprintConfig(config);
  return config;
}

SharedMemoryLayout SharedMemoryLayout::FromConfig(
    const SharedMemoryConfig& config,
    const ModelDimensions& dimensions) {
  SharedMemoryLayout layout;
  layout.dimensions = dimensions;
  layout.total_size = kArraysOffset;

  for (const FieldSpec& field : config.state_fields) {
    const std::uint32_t size = ResolveSize(field, dimensions);
    layout.state_index[field.name] = layout.state_fields.size();
    layout.state_fields.push_back({field.name, layout.total_size, size});
    layout.total_size += static_cast<std::size_t>(size) * kDoubleSize;
  }

  for (const FieldSpec& field : config.command_fields) {
    const std::uint32_t size = ResolveSize(field, dimensions);
    layout.command_index[field.name] = layout.command_fields.size();
    layout.command_fields.push_back({field.name, layout.total_size, size});
    layout.total_size += static_cast<std::size_t>(size) * kDoubleSize;
  }

  return layout;
}

RobotSharedMemory::RobotSharedMemory(RobotSharedMemory&& other) noexcept {
  *this = std::move(other);
}

RobotSharedMemory& RobotSharedMemory::operator=(
    RobotSharedMemory&& other) noexcept {
  if (this == &other) {
    return *this;
  }
  Close();
  fd_ = other.fd_;
  memory_ = other.memory_;
  mapped_size_ = other.mapped_size_;
  owner_ = other.owner_;
  unlink_on_close_ = other.unlink_on_close_;
  name_ = std::move(other.name_);
  posix_name_ = std::move(other.posix_name_);
  config_ = std::move(other.config_);
  layout_ = std::move(other.layout_);

  other.fd_ = -1;
  other.memory_ = nullptr;
  other.mapped_size_ = 0;
  other.owner_ = false;
  other.unlink_on_close_ = false;
  return *this;
}

RobotSharedMemory::~RobotSharedMemory() {
  try {
    Close();
  } catch (...) {
  }
}

RobotSharedMemory RobotSharedMemory::Create(
    const ModelDimensions& dimensions,
    double timestep,
    const std::string& config_path,
    const std::string& name_override,
    bool unlink_existing,
    bool unlink_on_close) {
  RobotSharedMemory io;
  io.config_ = SharedMemoryConfig::Load(config_path);
  io.layout_ = SharedMemoryLayout::FromConfig(io.config_, dimensions);
  io.name_ = name_override.empty() ? io.config_.shared_memory_name : name_override;
  io.posix_name_ = MakePosixName(io.name_);
  io.owner_ = true;
  io.unlink_on_close_ = unlink_on_close;

  io.fd_ = shm_open(io.posix_name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
  if (io.fd_ < 0 && errno == EEXIST && unlink_existing) {
    if (shm_unlink(io.posix_name_.c_str()) != 0) {
      ThrowErrno("failed to unlink existing shared memory");
    }
    io.fd_ = shm_open(io.posix_name_.c_str(), O_RDWR | O_CREAT | O_EXCL, 0600);
  }
  if (io.fd_ < 0) {
    ThrowErrno("failed to create shared memory");
  }
  if (ftruncate(io.fd_, static_cast<off_t>(io.layout_.total_size)) != 0) {
    ThrowErrno("failed to size shared memory");
  }

  io.mapped_size_ = io.layout_.total_size;
  void* mapped = mmap(
      nullptr,
      io.mapped_size_,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      io.fd_,
      0);
  if (mapped == MAP_FAILED) {
    ThrowErrno("failed to map shared memory");
  }
  io.memory_ = static_cast<std::uint8_t*>(mapped);

  const double now = WallTimeSeconds();
  io.WriteCommonHeader();
  io.WriteStateHeader(0, true, 0.0, timestep, now);
  io.WriteCommandHeader(0, false, kCommandModeTorque, now);
  io.ZeroArrays();
  return io;
}

RobotSharedMemory RobotSharedMemory::Attach(
    const std::string& config_path,
    const std::string& name_override) {
  RobotSharedMemory io;
  io.config_ = SharedMemoryConfig::Load(config_path);
  io.name_ = name_override.empty() ? io.config_.shared_memory_name : name_override;
  io.posix_name_ = MakePosixName(io.name_);

  io.fd_ = shm_open(io.posix_name_.c_str(), O_RDWR, 0600);
  if (io.fd_ < 0) {
    ThrowErrno("failed to attach shared memory");
  }

  struct stat status {};
  if (fstat(io.fd_, &status) != 0) {
    ThrowErrno("failed to stat shared memory");
  }
  if (status.st_size < static_cast<off_t>(kArraysOffset)) {
    throw std::runtime_error("shared-memory block is too small for headers");
  }
  io.mapped_size_ = static_cast<std::size_t>(status.st_size);
  void* mapped = mmap(
      nullptr,
      io.mapped_size_,
      PROT_READ | PROT_WRITE,
      MAP_SHARED,
      io.fd_,
      0);
  if (mapped == MAP_FAILED) {
    ThrowErrno("failed to map shared memory");
  }
  io.memory_ = static_cast<std::uint8_t*>(mapped);

  const CommonHeaderRaw common = ReadCommonHeaderRaw(io.memory_);
  if (common.config_fingerprint != io.config_.fingerprint) {
    throw std::runtime_error(
        "shared-memory config fingerprint mismatch; use the same JSON config");
  }
  const ModelDimensions dimensions{
      common.nq,
      common.nv,
      common.nu,
      common.nsensordata,
  };
  io.layout_ = SharedMemoryLayout::FromConfig(io.config_, dimensions);
  if (io.layout_.total_size != common.total_size) {
    throw std::runtime_error("shared-memory size mismatch");
  }
  if (io.mapped_size_ < io.layout_.total_size) {
    throw std::runtime_error("shared-memory block is smaller than configured layout");
  }
  return io;
}

SharedMemoryHeader RobotSharedMemory::ReadHeader() const {
  const CommonHeaderRaw common = ReadCommonHeaderRaw(memory_);
  const StateHeaderRaw state = ReadStateHeaderRaw(memory_);
  const CommandHeaderRaw command = ReadCommandHeaderRaw(memory_);

  SharedMemoryHeader header;
  header.total_size = common.total_size;
  header.config_fingerprint = common.config_fingerprint;
  header.dimensions = {
      common.nq,
      common.nv,
      common.nu,
      common.nsensordata,
  };
  header.state_seq = state.sequence;
  header.command_seq = command.sequence;
  header.sim_alive = state.sim_alive != 0;
  header.command_enabled = command.enabled != 0;
  header.command_mode = command.mode;
  header.sim_time = state.sim_time;
  header.timestep = state.timestep;
  header.state_wall_time = state.wall_time;
  header.command_wall_time = command.wall_time;
  return header;
}

void RobotSharedMemory::SetAlive(bool alive) {
  const StateHeaderRaw state = ReadStateHeaderRaw(memory_);
  WriteStateHeader(
      state.sequence,
      alive,
      state.sim_time,
      state.timestep,
      state.wall_time);
}

void RobotSharedMemory::WriteState(
    double sim_time,
    double timestep,
    const std::map<std::string, std::vector<double>>& fields) {
  ValidateFields(fields, layout_.state_fields, layout_.state_index, "state");
  const StateHeaderRaw state = ReadStateHeaderRaw(memory_);
  const std::uint64_t odd_sequence = NextOddSequence(state.sequence);
  WriteStateHeader(
      odd_sequence,
      state.sim_alive != 0,
      state.sim_time,
      state.timestep,
      state.wall_time);
  WriteFields(fields, layout_.state_fields);
  WriteStateHeader(odd_sequence + 1, true, sim_time, timestep, WallTimeSeconds());
}

RobotState RobotSharedMemory::ReadState(
    std::chrono::duration<double> timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const StateHeaderRaw before = ReadStateHeaderRaw(memory_);
    if (before.sequence % 2 != 0) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("timed out waiting for a stable state write");
      }
      std::this_thread::yield();
      continue;
    }

    auto fields = ReadFields(layout_.state_fields);
    const StateHeaderRaw after = ReadStateHeaderRaw(memory_);
    if (before.sequence == after.sequence && after.sequence % 2 == 0) {
      return RobotState{
          after.sequence,
          after.sim_alive != 0,
          after.sim_time,
          after.timestep,
          after.wall_time,
          std::move(fields),
      };
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("timed out waiting for a stable state read");
    }
  }
}

void RobotSharedMemory::WriteCommand(
    bool enabled,
    std::uint32_t mode,
    const std::map<std::string, std::vector<double>>& fields) {
  ValidateFields(fields, layout_.command_fields, layout_.command_index, "command");
  const CommandHeaderRaw command = ReadCommandHeaderRaw(memory_);
  const std::uint64_t odd_sequence = NextOddSequence(command.sequence);
  WriteCommandHeader(
      odd_sequence,
      command.enabled != 0,
      command.mode,
      command.wall_time);
  WriteFields(fields, layout_.command_fields);
  WriteCommandHeader(odd_sequence + 1, enabled, mode, WallTimeSeconds());
}

void RobotSharedMemory::WriteTorque(
    const std::vector<double>& torque,
    bool enabled) {
  if (layout_.command_index.find("torque") == layout_.command_index.end()) {
    throw std::runtime_error("command field 'torque' is not configured");
  }
  std::map<std::string, std::vector<double>> fields;
  for (const FieldLayout& field : layout_.command_fields) {
    fields[field.name] = std::vector<double>(field.size, 0.0);
  }
  fields["torque"] = torque;
  WriteCommand(enabled, kCommandModeTorque, fields);
}

void RobotSharedMemory::DisableCommand() {
  std::map<std::string, std::vector<double>> fields;
  for (const FieldLayout& field : layout_.command_fields) {
    fields[field.name] = std::vector<double>(field.size, 0.0);
  }
  WriteCommand(false, kCommandModeTorque, fields);
}

RobotCommand RobotSharedMemory::ReadCommand(
    std::chrono::duration<double> timeout) const {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const CommandHeaderRaw before = ReadCommandHeaderRaw(memory_);
    if (before.sequence % 2 != 0) {
      if (std::chrono::steady_clock::now() >= deadline) {
        throw std::runtime_error("timed out waiting for a stable command write");
      }
      std::this_thread::yield();
      continue;
    }

    auto fields = ReadFields(layout_.command_fields);
    const CommandHeaderRaw after = ReadCommandHeaderRaw(memory_);
    if (before.sequence == after.sequence && after.sequence % 2 == 0) {
      return RobotCommand{
          after.sequence,
          after.enabled != 0,
          after.mode,
          after.wall_time,
          std::move(fields),
      };
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error("timed out waiting for a stable command read");
    }
  }
}

void RobotSharedMemory::Close() {
  if (memory_) {
    if (owner_) {
      try {
        SetAlive(false);
      } catch (...) {
      }
    }
    if (owner_ && unlink_on_close_) {
      shm_unlink(posix_name_.c_str());
    }
    munmap(memory_, mapped_size_);
    memory_ = nullptr;
    mapped_size_ = 0;
  }
  if (fd_ >= 0) {
    close(fd_);
    fd_ = -1;
  }
  owner_ = false;
}

void RobotSharedMemory::WriteCommonHeader() const {
  CommonHeaderRaw header {};
  std::memcpy(header.magic, kMagic, sizeof(header.magic));
  header.version = kVersion;
  header.arrays_offset = static_cast<std::uint32_t>(kArraysOffset);
  header.total_size = layout_.total_size;
  header.nq = layout_.dimensions.nq;
  header.nv = layout_.dimensions.nv;
  header.nu = layout_.dimensions.nu;
  header.nsensordata = layout_.dimensions.nsensordata;
  header.config_fingerprint = config_.fingerprint;
  WriteStruct(memory_, kCommonHeaderOffset, header);
}

void RobotSharedMemory::WriteStateHeader(
    std::uint64_t sequence,
    bool sim_alive,
    double sim_time,
    double timestep,
    double wall_time) const {
  const StateHeaderRaw header{
      sequence,
      static_cast<std::uint32_t>(sim_alive ? 1 : 0),
      0,
      sim_time,
      timestep,
      wall_time,
  };
  WriteStruct(memory_, kStateHeaderOffset, header);
}

void RobotSharedMemory::WriteCommandHeader(
    std::uint64_t sequence,
    bool enabled,
    std::uint32_t mode,
    double wall_time) const {
  const CommandHeaderRaw header{
      sequence,
      static_cast<std::uint32_t>(enabled ? 1 : 0),
      mode,
      wall_time,
  };
  WriteStruct(memory_, kCommandHeaderOffset, header);
}

void RobotSharedMemory::ZeroArrays() const {
  std::memset(memory_ + kArraysOffset, 0, layout_.total_size - kArraysOffset);
}

void RobotSharedMemory::ValidateFields(
    const std::map<std::string, std::vector<double>>& fields,
    const std::vector<FieldLayout>& layouts,
    const std::map<std::string, std::size_t>& index,
    const std::string& label) const {
  for (const FieldLayout& field : layouts) {
    const auto it = fields.find(field.name);
    if (it == fields.end()) {
      throw std::runtime_error("missing " + label + " field: " + field.name);
    }
    if (it->second.size() != field.size) {
      throw std::runtime_error(
          label + " field " + field.name + " must contain " +
          std::to_string(field.size) + " values, got " +
          std::to_string(it->second.size()));
    }
  }
  for (const auto& entry : fields) {
    if (index.find(entry.first) == index.end()) {
      throw std::runtime_error("unknown " + label + " field: " + entry.first);
    }
  }
}

void RobotSharedMemory::WriteFields(
    const std::map<std::string, std::vector<double>>& fields,
    const std::vector<FieldLayout>& layouts) const {
  for (const FieldLayout& field : layouts) {
    const std::vector<double>& values = fields.at(field.name);
    if (!values.empty()) {
      std::memcpy(
          memory_ + field.offset,
          values.data(),
          values.size() * sizeof(double));
    }
  }
}

std::map<std::string, std::vector<double>> RobotSharedMemory::ReadFields(
    const std::vector<FieldLayout>& layouts) const {
  std::map<std::string, std::vector<double>> fields;
  for (const FieldLayout& field : layouts) {
    std::vector<double> values(field.size);
    if (!values.empty()) {
      std::memcpy(
          values.data(),
          memory_ + field.offset,
          values.size() * sizeof(double));
    }
    fields[field.name] = std::move(values);
  }
  return fields;
}

}  // namespace robot_sim
