#pragma once

#include "json.h"

#include <chrono>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace session_cpp {
namespace mini_test {

struct CheckRecord {
  int line = -1;
  std::string code_line;
  bool passed = false;
};

struct TestResult {
  std::string group;
  std::string test_name;
  bool passed = false;
  double time_ms = 0.0;  // includes assertions
  int line = -1;
  std::string code;       // left empty for C++ version
  std::vector<CheckRecord> checks;
  nlohmann::ordered_json failures;  // array of failure objects
};

using TestFunc = std::function<void()>;

class MiniTestAssertionError : public std::runtime_error {
public:
  explicit MiniTestAssertionError(const std::string &msg)
      : std::runtime_error(msg) {}
};

// Registration and runtime hooks
void register_test(const std::string &group, const std::string &test_name,
                   const char *file, int line, TestFunc func);

void record_check(bool passed, int line, const char *expr_text);

void run_all(const std::string &language = "cpp");

// Macros for defining tests and checks
// C++ version: use string literals for group and name, similar to Python
// MINI_TEST("Point", "constructor") {
//   ...
// }
// We generate a unique internal function name using __LINE__.
#define MINI_TEST(GROUP_STR, NAME_STR)                                           \
  static void mini_test_fn_line_##__LINE__();                                   \
  static bool mini_test_reg_line_##__LINE__ = []() {                            \
    ::session_cpp::mini_test::register_test(GROUP_STR, NAME_STR, __FILE__,      \
                                            __LINE__,                           \
                                            mini_test_fn_line_##__LINE__);      \
    return true;                                                                \
  }();                                                                          \
  static void mini_test_fn_line_##__LINE__()

#define MINI_CHECK(EXPR)                                                         \
  do {                                                                          \
    bool _mt_passed = static_cast<bool>(EXPR);                                  \
    ::session_cpp::mini_test::record_check(_mt_passed, __LINE__, #EXPR);        \
    if (!_mt_passed) {                                                          \
      throw ::session_cpp::mini_test::MiniTestAssertionError(#EXPR);            \
    }                                                                           \
  } while (false)

} // namespace mini_test
} // namespace session_cpp
