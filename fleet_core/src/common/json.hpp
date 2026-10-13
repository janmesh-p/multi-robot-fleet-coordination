#pragma once

// Private to fleet_core: a small strict JSON reader for model files (RFC 8259, nothing more).
#include <cstddef>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fleet_core::json {

class ParseError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

struct Value {
  enum class Type { kNull, kBool, kNumber, kString, kArray, kObject };

  Type type{Type::kNull};
  bool boolean{false};
  double number{0.0};  // may be infinite when the literal overflows a double
  std::string text{};  // kString
  std::vector<Value> items{};
  std::vector<std::pair<std::string, Value>> members{};  // file order, keys unique

  const Value* find(std::string_view key) const;
};

// Parses one JSON document. Throws ParseError with the line and column for anything RFC 8259
// does not allow, including NaN and Infinity, trailing commas, comments, and duplicate keys.
Value parse(std::string_view text);

}  // namespace fleet_core::json
