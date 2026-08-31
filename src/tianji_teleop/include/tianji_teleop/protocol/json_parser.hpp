#pragma once

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>
#include <string_view>
#include <vector>

namespace tianji_teleop::protocol {

class JsonValue {
public:
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

  JsonValue() : type_(Type::kNull) {}
  explicit JsonValue(bool value) : type_(Type::kBool), bool_(value) {}
  explicit JsonValue(double value, bool integer, std::string lexical)
  : type_(Type::kNumber), number_(value), integer_(integer), lexical_(std::move(lexical)) {}
  explicit JsonValue(std::string value) : type_(Type::kString), string_(std::move(value)) {}
  explicit JsonValue(std::vector<JsonValue> value) : type_(Type::kArray), array_(std::move(value)) {}
  explicit JsonValue(std::map<std::string, JsonValue> value) : type_(Type::kObject), object_(std::move(value)) {}

  Type type() const { return type_; }
  bool is_null() const { return type_ == Type::kNull; }
  bool is_number() const { return type_ == Type::kNumber; }
  bool is_string() const { return type_ == Type::kString; }
  bool is_array() const { return type_ == Type::kArray; }
  bool is_object() const { return type_ == Type::kObject; }
  bool as_bool() const {
    if (type_ != Type::kBool) throw std::invalid_argument("JSON value is not boolean");
    return bool_;
  }
  double as_number() const {
    if (type_ != Type::kNumber || !std::isfinite(number_)) throw std::invalid_argument("JSON value is not finite number");
    return number_;
  }
  std::uint64_t as_uint(const std::string &field) const {
    if (type_ != Type::kNumber || !integer_ || lexical_.empty() || lexical_.front() == '-') {
      throw std::invalid_argument(field + " must be a non-negative integer");
    }
    try {
      std::size_t used = 0;
      const auto value = std::stoull(lexical_, &used);
      if (used != lexical_.size()) throw std::invalid_argument("invalid integer");
      return value;
    } catch (const std::exception &) {
      throw std::invalid_argument(field + " must be a non-negative integer");
    }
  }
  const std::string &as_string(const std::string &field) const {
    if (type_ != Type::kString || string_.empty()) throw std::invalid_argument(field + " must be a non-empty string");
    return string_;
  }
  const std::vector<JsonValue> &as_array(const std::string &field) const {
    if (type_ != Type::kArray) throw std::invalid_argument(field + " must be an array");
    return array_;
  }
  const std::map<std::string, JsonValue> &as_object(const std::string &field) const {
    if (type_ != Type::kObject) throw std::invalid_argument(field + " must be an object");
    return object_;
  }

private:
  Type type_;
  bool bool_{false};
  double number_{0.0};
  bool integer_{false};
  std::string lexical_;
  std::string string_;
  std::vector<JsonValue> array_;
  std::map<std::string, JsonValue> object_;
};

class StrictJsonParser {
public:
  static JsonValue parse(std::string_view text) {
    StrictJsonParser parser(text);
    auto result = parser.value();
    parser.space();
    if (parser.pos_ != parser.text_.size()) throw std::invalid_argument("trailing JSON data");
    return result;
  }

private:
  explicit StrictJsonParser(std::string_view text) : text_(text) {}

  void space() {
    while (pos_ < text_.size()) {
      const unsigned char c = static_cast<unsigned char>(text_[pos_]);
      if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
      ++pos_;
    }
  }
  char take() {
    if (pos_ >= text_.size()) throw std::invalid_argument("unexpected end of JSON");
    return text_[pos_++];
  }
  void expect(char expected) {
    if (take() != expected) throw std::invalid_argument("invalid JSON delimiter");
  }
  JsonValue value() {
    space();
    if (pos_ >= text_.size()) throw std::invalid_argument("missing JSON value");
    switch (text_[pos_]) {
      case 'n': literal("null"); return JsonValue();
      case 't': literal("true"); return JsonValue(true);
      case 'f': literal("false"); return JsonValue(false);
      case '"': return JsonValue(string_value());
      case '[': return array_value();
      case '{': return object_value();
      default: return number_value();
    }
  }
  void literal(std::string_view expected) {
    if (text_.substr(pos_, expected.size()) != expected) throw std::invalid_argument("invalid JSON literal");
    pos_ += expected.size();
  }
  static int hex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }
  static void utf8(std::string &out, unsigned codepoint) {
    if (codepoint <= 0x7f) out.push_back(static_cast<char>(codepoint));
    else if (codepoint <= 0x7ff) {
      out.push_back(static_cast<char>(0xc0 | (codepoint >> 6)));
      out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else if (codepoint <= 0xffff) {
      out.push_back(static_cast<char>(0xe0 | (codepoint >> 12)));
      out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
      out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    } else {
      out.push_back(static_cast<char>(0xf0 | (codepoint >> 18)));
      out.push_back(static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f)));
      out.push_back(static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f)));
      out.push_back(static_cast<char>(0x80 | (codepoint & 0x3f)));
    }
  }
  std::string string_value() {
    expect('"');
    std::string result;
    while (pos_ < text_.size()) {
      const unsigned char c = static_cast<unsigned char>(take());
      if (c == '"') return result;
      if (c < 0x20) throw std::invalid_argument("control character in JSON string");
      if (c != '\\') { result.push_back(static_cast<char>(c)); continue; }
      const char escaped = take();
      switch (escaped) {
        case '"': case '\\': case '/': result.push_back(escaped); break;
        case 'b': result.push_back('\b'); break;
        case 'f': result.push_back('\f'); break;
        case 'n': result.push_back('\n'); break;
        case 'r': result.push_back('\r'); break;
        case 't': result.push_back('\t'); break;
        case 'u': {
          unsigned codepoint = 0;
          for (int i = 0; i < 4; ++i) {
            const int digit = pos_ < text_.size() ? hex(text_[pos_++]) : -1;
            if (digit < 0) throw std::invalid_argument("invalid unicode escape");
            codepoint = (codepoint << 4) | static_cast<unsigned>(digit);
          }
          utf8(result, codepoint);
          break;
        }
        default: throw std::invalid_argument("invalid JSON escape");
      }
    }
    throw std::invalid_argument("unterminated JSON string");
  }
  JsonValue number_value() {
    const auto start = pos_;
    if (pos_ < text_.size() && text_[pos_] == '-') ++pos_;
    if (pos_ >= text_.size()) throw std::invalid_argument("invalid JSON number");
    if (text_[pos_] == '0') {
      ++pos_;
      if (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') throw std::invalid_argument("leading zero in JSON number");
    } else {
      if (text_[pos_] < '1' || text_[pos_] > '9') throw std::invalid_argument("invalid JSON number");
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
    }
    bool integer = true;
    if (pos_ < text_.size() && text_[pos_] == '.') {
      integer = false; ++pos_;
      if (pos_ >= text_.size() || text_[pos_] < '0' || text_[pos_] > '9') throw std::invalid_argument("invalid JSON fraction");
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
    }
    if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
      integer = false; ++pos_;
      if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) ++pos_;
      if (pos_ >= text_.size() || text_[pos_] < '0' || text_[pos_] > '9') throw std::invalid_argument("invalid JSON exponent");
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
    }
    const std::string lexical(text_.substr(start, pos_ - start));
    errno = 0;
    char *end = nullptr;
    const double number = std::strtod(lexical.c_str(), &end);
    if (errno == ERANGE || end != lexical.c_str() + lexical.size() || !std::isfinite(number)) throw std::invalid_argument("non-finite JSON number");
    return JsonValue(number, integer, lexical);
  }
  JsonValue array_value() {
    expect('['); space();
    std::vector<JsonValue> result;
    if (pos_ < text_.size() && text_[pos_] == ']') { ++pos_; return JsonValue(std::move(result)); }
    while (true) {
      result.push_back(value()); space();
      const char delimiter = take();
      if (delimiter == ']') break;
      if (delimiter != ',') throw std::invalid_argument("invalid JSON array delimiter");
      space();
      if (pos_ < text_.size() && text_[pos_] == ']') throw std::invalid_argument("trailing comma in JSON array");
    }
    return JsonValue(std::move(result));
  }
  JsonValue object_value() {
    expect('{'); space();
    std::map<std::string, JsonValue> result;
    if (pos_ < text_.size() && text_[pos_] == '}') { ++pos_; return JsonValue(std::move(result)); }
    while (true) {
      if (pos_ >= text_.size() || text_[pos_] != '"') throw std::invalid_argument("JSON object key must be string");
      std::string key = string_value(); space(); expect(':');
      auto [it, inserted] = result.emplace(key, value());
      if (!inserted) throw std::invalid_argument("duplicate JSON object field: " + key);
      space();
      const char delimiter = take();
      if (delimiter == '}') break;
      if (delimiter != ',') throw std::invalid_argument("invalid JSON object delimiter");
      space();
      if (pos_ < text_.size() && text_[pos_] == '}') throw std::invalid_argument("trailing comma in JSON object");
    }
    return JsonValue(std::move(result));
  }

  std::string_view text_;
  std::size_t pos_{0};
};

inline const JsonValue &field(const JsonValue &object, const std::string &name) {
  const auto &values = object.as_object("message");
  const auto it = values.find(name);
  if (it == values.end()) throw std::invalid_argument("missing field " + name);
  return it->second;
}

inline void require_exact_fields(const JsonValue &object, std::initializer_list<std::string_view> expected) {
  const auto &values = object.as_object("message");
  std::set<std::string> names;
  for (const auto name : expected) names.emplace(name);
  if (values.size() != names.size()) throw std::invalid_argument("protocol fields mismatch");
  for (const auto &entry : values) {
    if (names.find(entry.first) == names.end()) throw std::invalid_argument("unknown protocol field " + entry.first);
  }
}

inline std::vector<double> vector_field(const JsonValue &object, const std::string &name, std::size_t size) {
  const auto &values = field(object, name).as_array(name);
  if (values.size() != size) throw std::invalid_argument(name + " has invalid shape");
  std::vector<double> result;
  result.reserve(size);
  for (std::size_t index = 0; index < size; ++index) result.push_back(values[index].as_number());
  return result;
}

inline std::vector<std::string> string_array_field(const JsonValue &object, const std::string &name, std::size_t size) {
  const auto &values = field(object, name).as_array(name);
  if (values.size() != size) throw std::invalid_argument(name + " has invalid shape");
  std::vector<std::string> result;
  result.reserve(size);
  for (std::size_t index = 0; index < size; ++index) result.push_back(values[index].as_string(name));
  return result;
}

}  // namespace tianji_teleop::protocol
