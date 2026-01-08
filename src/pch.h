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

// STL misc
#include <stdexcept>
#include <cassert>
#include <cstdint>

#endif // PCH_H
