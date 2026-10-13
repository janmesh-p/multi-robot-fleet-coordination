#include "json.hpp"

#include <cstdint>
#include <cstdlib>
#include <set>

namespace fleet_core::json {
namespace {

constexpr int kMaxDepth = 64;

class Parser {
 public:
  explicit Parser(std::string_view text) : text_(text) {}

  Value document() {
    skipSpace();
    Value v = value(0);
    skipSpace();
    if (pos_ != text_.size()) fail("unexpected text after the document");
    return v;
  }

 private:
  [[noreturn]] void fail(const std::string& what) const {
    size_t line = 1, column = 1;
    for (size_t i = 0; i < pos_ && i < text_.size(); ++i) {
      if (text_[i] == '\n') {
        ++line;
        column = 1;
      } else {
        ++column;
      }
    }
    throw ParseError("invalid JSON at line " + std::to_string(line) + ", column " +
                     std::to_string(column) + ": " + what);
  }

  bool atEnd() const { return pos_ >= text_.size(); }
  char peek() const { return atEnd() ? '\0' : text_[pos_]; }

  void skipSpace() {
    while (!atEnd()) {
      const char c = text_[pos_];
      if (c != ' ' && c != '\t' && c != '\n' && c != '\r') break;
      ++pos_;
    }
  }

  void expect(char c) {
    if (peek() != c) fail(std::string("expected '") + c + "'");
    ++pos_;
  }

  void literal(std::string_view word) {
    if (text_.substr(pos_, word.size()) != word) fail("unknown literal");
    pos_ += word.size();
  }

  Value value(int depth) {
    if (depth > kMaxDepth) fail("nested too deeply");
    Value v;
    switch (peek()) {
      case '{':
        return object(depth);
      case '[':
        return array(depth);
      case '"':
        v.type = Value::Type::kString;
        v.text = string();
        return v;
      case 't':
        literal("true");
        v.type = Value::Type::kBool;
        v.boolean = true;
        return v;
      case 'f':
        literal("false");
        v.type = Value::Type::kBool;
        return v;
      case 'n':
        literal("null");
        return v;
      default:
        if (peek() == '-' || (peek() >= '0' && peek() <= '9')) return number();
        fail(atEnd() ? "unexpected end of input" : "expected a value");
    }
  }

  Value object(int depth) {
    Value v;
    v.type = Value::Type::kObject;
    expect('{');
    skipSpace();
    if (peek() == '}') {
      ++pos_;
      return v;
    }
    std::set<std::string> keys;
    while (true) {
      skipSpace();
      if (peek() != '"') fail("expected a string key");
      const size_t key_pos = pos_;
      std::string key = string();
      if (!keys.insert(key).second) {
        pos_ = key_pos;
        fail("duplicate key \"" + key + "\"");
      }
      skipSpace();
      expect(':');
      skipSpace();
      Value member = value(depth + 1);
      v.members.emplace_back(std::move(key), std::move(member));
      skipSpace();
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      expect('}');
      return v;
    }
  }

  Value array(int depth) {
    Value v;
    v.type = Value::Type::kArray;
    expect('[');
    skipSpace();
    if (peek() == ']') {
      ++pos_;
      return v;
    }
    while (true) {
      skipSpace();
      v.items.push_back(value(depth + 1));
      skipSpace();
      if (peek() == ',') {
        ++pos_;
        continue;
      }
      expect(']');
      return v;
    }
  }

  static void appendUtf8(std::string& out, uint32_t cp) {
    if (cp < 0x80) {
      out += static_cast<char>(cp);
    } else if (cp < 0x800) {
      out += static_cast<char>(0xC0 | (cp >> 6));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
      out += static_cast<char>(0xE0 | (cp >> 12));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
      out += static_cast<char>(0xF0 | (cp >> 18));
      out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    }
  }

  uint32_t hex4() {
    uint32_t cp = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = peek();
      uint32_t digit = 0;
      if (c >= '0' && c <= '9') {
        digit = static_cast<uint32_t>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        digit = static_cast<uint32_t>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        digit = static_cast<uint32_t>(c - 'A' + 10);
      } else {
        fail("expected four hex digits after \\u");
      }
      cp = cp * 16 + digit;
      ++pos_;
    }
    return cp;
  }

  std::string string() {
    expect('"');
    std::string out;
    while (true) {
      if (atEnd()) fail("unterminated string");
      const char c = text_[pos_];
      if (c == '"') {
        ++pos_;
        return out;
      }
      if (static_cast<unsigned char>(c) < 0x20) fail("control character in a string");
      if (c != '\\') {
        out += c;
        ++pos_;
        continue;
      }
      ++pos_;
      const char e = peek();
      ++pos_;
      switch (e) {
        case '"':
        case '\\':
        case '/':
          out += e;
          break;
        case 'b':
          out += '\b';
          break;
        case 'f':
          out += '\f';
          break;
        case 'n':
          out += '\n';
          break;
        case 'r':
          out += '\r';
          break;
        case 't':
          out += '\t';
          break;
        case 'u': {
          uint32_t cp = hex4();
          if (cp >= 0xD800 && cp <= 0xDBFF) {  // high surrogate: a low one must follow
            if (peek() != '\\') fail("unpaired surrogate");
            ++pos_;
            if (peek() != 'u') fail("unpaired surrogate");
            ++pos_;
            const uint32_t low = hex4();
            if (low < 0xDC00 || low > 0xDFFF) fail("unpaired surrogate");
            cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
          } else if (cp >= 0xDC00 && cp <= 0xDFFF) {
            fail("unpaired surrogate");
          }
          appendUtf8(out, cp);
          break;
        }
        default:
          --pos_;
          fail("unknown escape");
      }
    }
  }

  Value number() {
    const size_t start = pos_;
    const auto digits = [&] {
      const size_t from = pos_;
      while (peek() >= '0' && peek() <= '9') ++pos_;
      return pos_ - from;
    };
    if (peek() == '-') ++pos_;
    if (peek() == '0') {
      ++pos_;
    } else if (digits() == 0) {
      fail("expected a digit");
    }
    if (peek() == '.') {
      ++pos_;
      if (digits() == 0) fail("expected a digit after '.'");
    }
    if (peek() == 'e' || peek() == 'E') {
      ++pos_;
      if (peek() == '+' || peek() == '-') ++pos_;
      if (digits() == 0) fail("expected a digit in the exponent");
    }
    const std::string literal_text(text_.substr(start, pos_ - start));
    Value v;
    v.type = Value::Type::kNumber;
    v.number = std::strtod(literal_text.c_str(), nullptr);  // "C" locale: '.' is the point
    return v;
  }

  std::string_view text_;
  size_t pos_{0};
};

}  // namespace

const Value* Value::find(std::string_view key) const {
  for (const auto& member : members) {
    if (member.first == key) return &member.second;
  }
  return nullptr;
}

Value parse(std::string_view text) { return Parser(text).document(); }

}  // namespace fleet_core::json
