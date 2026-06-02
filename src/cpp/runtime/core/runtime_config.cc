#include "core/runtime_config.hpp"

#include <cctype>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace robot_runtime {
namespace {

std::string ReadTextFile(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("failed to open runtime config: " + path);
  }
  return std::string(
      std::istreambuf_iterator<char>(file),
      std::istreambuf_iterator<char>());
}

struct JsonValue {
  enum class Type {
    kNull,
    kObject,
    kArray,
    kString,
    kInteger,
    kBoolean,
  };

  Type type = Type::kNull;
  std::map<std::string, JsonValue> object;
  std::vector<JsonValue> array;
  std::string string;
  long long integer = 0;
  bool boolean = false;
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
    if (text_.compare(position_, 4, "true") == 0) {
      position_ += 4;
      JsonValue value;
      value.type = JsonValue::Type::kBoolean;
      value.boolean = true;
      return value;
    }
    if (text_.compare(position_, 5, "false") == 0) {
      position_ += 5;
      JsonValue value;
      value.type = JsonValue::Type::kBoolean;
      value.boolean = false;
      return value;
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
      throw std::runtime_error("floating-point numbers are not supported here");
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
  RequireObject(object, "runtime config");
  const auto it = object.object.find(key);
  if (it == object.object.end()) {
    return nullptr;
  }
  return &it->second;
}

std::string RequiredString(const JsonValue& object, const std::string& key) {
  const JsonValue* value = FindKey(object, key);
  if (!value || value->type != JsonValue::Type::kString || value->string.empty()) {
    throw std::runtime_error(key + " must be a non-empty string");
  }
  return value->string;
}

bool OptionalBool(const JsonValue& object, const std::string& key, bool fallback) {
  const JsonValue* value = FindKey(object, key);
  if (!value) {
    return fallback;
  }
  if (value->type != JsonValue::Type::kBoolean) {
    throw std::runtime_error(key + " must be a boolean");
  }
  return value->boolean;
}

std::string ScalarToString(const JsonValue& value) {
  if (value.type == JsonValue::Type::kString) {
    return value.string;
  }
  if (value.type == JsonValue::Type::kInteger) {
    return std::to_string(value.integer);
  }
  if (value.type == JsonValue::Type::kBoolean) {
    return value.boolean ? "true" : "false";
  }
  throw std::runtime_error("plugin config values must be scalar");
}

std::map<std::string, std::string> OptionalStringMap(
    const JsonValue& object,
    const std::string& key) {
  const JsonValue* value = FindKey(object, key);
  if (!value) {
    return {};
  }
  RequireObject(*value, key);
  std::map<std::string, std::string> result;
  for (const auto& entry : value->object) {
    result[entry.first] = ScalarToString(entry.second);
  }
  return result;
}

}  // namespace

RuntimeConfig RuntimeConfig::Load(const std::string& path) {
  JsonParser parser(ReadTextFile(path));
  const JsonValue root = parser.Parse();
  RequireObject(root, "runtime config");

  const JsonValue* plugins = FindKey(root, "plugins");
  if (!plugins || plugins->type != JsonValue::Type::kArray) {
    throw std::runtime_error("plugins must be a JSON array");
  }

  RuntimeConfig config;
  for (const JsonValue& raw_plugin : plugins->array) {
    RequireObject(raw_plugin, "plugin");
    PluginSpec spec;
    spec.name = RequiredString(raw_plugin, "name");
    spec.type = RequiredString(raw_plugin, "type");
    spec.enabled = OptionalBool(raw_plugin, "enabled", true);
    spec.config = OptionalStringMap(raw_plugin, "config");
    config.plugins.push_back(std::move(spec));
  }
  return config;
}

}  // namespace robot_runtime
