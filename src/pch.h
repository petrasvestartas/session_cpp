#ifndef PCH_H
#define PCH_H

// ============================================================================
// PRECOMPILED HEADER (PCH)
// ============================================================================
// This file contains stable STL headers that rarely change.
// Including them here speeds up incremental builds significantly.
//
// WORKFLOW:
// - Add only stable, rarely-changing headers here
// - Do NOT add project headers that are under active development
// - After modifying this file, delete build/CMakeFiles to force PCH rebuild
// ============================================================================

// STL containers
#include <vector>
#include <array>
#include <string>
#include <map>
#include <unordered_map>
#include <set>
#include <unordered_set>

// STL utilities
#include <memory>
#include <optional>
#include <variant>
#include <tuple>
#include <functional>
#include <utility>

// STL algorithms and numerics
#include <algorithm>
#include <numeric>
#include <cmath>
#include <limits>

// STL I/O
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
#include <chrono>

// STL misc
#include <stdexcept>
#include <cassert>
#include <cstdint>

// Vendored and third-party headers nearly every file parses (1.0 s of the 1.7 s front end per file)
#include "json.h"
#include "fmt/core.h"
#include <google/protobuf/message.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/map_field.h>

#endif // PCH_H
