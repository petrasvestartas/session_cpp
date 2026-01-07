#include "mini_test.h"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>

namespace session_cpp {
namespace mini_test {

namespace fs = std::filesystem;

struct RegisteredTest {
  std::string group;
  std::string name;
  std::string file;
  int line = -1;
  TestFunc func;
};

static std::vector<RegisteredTest> &registry() {
  static std::vector<RegisteredTest> tests;
  return tests;
}

static std::vector<CheckRecord> *&current_checks() {
  static std::vector<CheckRecord> *ptr = nullptr;
  return ptr;
}

static double &current_assertion_time() {
  static double t = 0.0;
  return t;
}

void register_test(const std::string &group, const std::string &test_name,
                   const char *file, int line, TestFunc func) {
  registry().push_back(RegisteredTest{group, test_name, file, line, std::move(func)});
}

void record_check(bool passed, int line, const char *expr_text,
                  std::chrono::high_resolution_clock::time_point check_start) {
  auto *checks = current_checks();
  if (!checks) {
    return;
  }
  checks->push_back(CheckRecord{line, std::string(expr_text), passed});
  auto check_end = std::chrono::high_resolution_clock::now();
  current_assertion_time() += std::chrono::duration<double, std::milli>(check_end - check_start).count();
}

static std::string extract_timed_code(const RegisteredTest &t,
                                      const std::vector<CheckRecord> &checks) {
  // Find the last check line to determine where the test body ends
  int last_check_line = 0;
  for (const auto &c : checks) {
    if (c.line > last_check_line) {
      last_check_line = c.line;
    }
  }

  int start_line = t.line + 1;  // line after MINI_TEST macro
  // End at last check line + some buffer for closing braces, or use a reasonable range
  int end_line = last_check_line > 0 ? last_check_line + 2 : start_line + 100;

  std::ifstream ifs(t.file);
  if (!ifs.is_open()) {
    return std::string();
  }

  std::string line;
  std::string code;
  int current = 1;
  bool in_check = false;
  int paren_depth = 0;

  while (std::getline(ifs, line)) {
    if (current >= start_line && current <= end_line) {
      // Check if this line starts a MINI_CHECK
      if (line.find("MINI_CHECK(") != std::string::npos) {
        in_check = true;
        paren_depth = 0;
        for (char ch : line) {
          if (ch == '(') paren_depth++;
          else if (ch == ')') paren_depth--;
        }
        if (paren_depth <= 0) in_check = false;
      } else if (in_check) {
        // Continue tracking multi-line MINI_CHECK
        for (char ch : line) {
          if (ch == '(') paren_depth++;
          else if (ch == ')') paren_depth--;
        }
        if (paren_depth <= 0) in_check = false;
      } else {
        // Not in a MINI_CHECK - include this line
        std::string trimmed = line;
        size_t first = trimmed.find_first_not_of(" \t");
        if (first != std::string::npos) {
          trimmed = trimmed.substr(first);
        }
        // Skip closing brace only lines
        if (trimmed != "}" && trimmed != "});") {
          if (!code.empty()) {
            code.push_back('\n');
          }
          code += line;
        }
      }
    }
    if (current > end_line) {
      break;
    }
    ++current;
  }

  return code;
}

static fs::path compute_repo_root() {
  fs::path here = fs::absolute(__FILE__);
  // __FILE__ is session_cpp/src/mini_test.cpp
  return here.parent_path().parent_path().parent_path();
}

void run_all(const std::string &language) {
  (void)language; // currently only "cpp" is used for folder selection

  auto &tests = registry();
  if (tests.empty()) {
    return;
  }

  // Group tests by logical group name
  std::map<std::string, std::vector<const RegisteredTest *>> tests_by_group;
  for (const auto &t : tests) {
    tests_by_group[t.group].push_back(&t);
  }

  fs::path repo_root = compute_repo_root();
  fs::path subdir = repo_root / "session_tests" / "session_cpp";

  for (const auto &[group, group_tests] : tests_by_group) {
    nlohmann::ordered_json results = nlohmann::json::array();
    fs::path test_file_path;

    for (const RegisteredTest *t : group_tests) {
      std::vector<CheckRecord> checks;
      current_checks() = &checks;
      current_assertion_time() = 0.0;

      bool passed = true;
      nlohmann::ordered_json failures = nlohmann::json::array();

      auto start = std::chrono::high_resolution_clock::now();
      try {
        t->func();
      } catch (const MiniTestAssertionError &e) {
        passed = false;
        failures.push_back(nlohmann::ordered_json{{"error", e.what()}});
      } catch (const std::exception &e) {
        passed = false;
        failures.push_back(nlohmann::ordered_json{{"error", e.what()}});
      } catch (...) {
        passed = false;
        failures.push_back(nlohmann::ordered_json{{"error", "unknown exception"}});
      }
      auto end = std::chrono::high_resolution_clock::now();
      current_checks() = nullptr;

      double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
      double effective_ms = std::max(0.0, elapsed_ms - current_assertion_time());
      double rounded_ms = std::round(effective_ms * 1000.0) / 1000.0; // three decimals

      if (test_file_path.empty()) {
        test_file_path = fs::path(t->file);
      }

      // Convert checks to JSON
      nlohmann::ordered_json checks_json = nlohmann::json::array();
      for (const auto &c : checks) {
        checks_json.push_back({{"line", c.line}, {"code_line", c.code_line}, {"passed", c.passed}});
      }

      std::string code_str = extract_timed_code(*t, checks);

      nlohmann::ordered_json result = {
          {"test_name", t->name},
          {"passed", passed},
          {"time_ms", rounded_ms},
          {"line", t->line},
          {"code", code_str},
          {"checks", checks_json},
          {"failures", failures},
      };

      results.push_back(std::move(result));
    }

    if (!test_file_path.empty()) {
      fs::path out_path = subdir / (test_file_path.stem().string() + ".json");
      fs::create_directories(out_path.parent_path());
      std::ofstream ofs(out_path);
      if (ofs.is_open()) {
        ofs << results.dump(2);
      }
    }
  }
}

} // namespace mini_test
} // namespace session_cpp
