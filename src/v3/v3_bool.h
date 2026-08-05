#pragma once

// v3 boolean driver (BOPAlgo_BOP equivalent). Pipeline per kb/BOOL_V3_MEMORY.md:
// orient -> bbox gate -> FF sections (once per face pair, shared by both
// operands) -> clip to face regions -> paves on edges -> split edges -> split
// faces (UV arrangement) -> classify parts (seeded once) -> select (op-table)
// -> assemble (shared section edges, 3D-hash vertex weld, no sewing).
#include "v3_topo.h"
#include "v3_split.h"

namespace v3 {

enum class BoolOp { FUSE, CUT, COMMON };

// boolean of two CLOSED solids. `intersected` (optional) reports whether the
// operands' boundaries actually interfered (user requirement: callers check).
Solid boolean(const Solid& A, const Solid& B, BoolOp op, double tol = 1e-6,
              bool* intersected = nullptr);

// BRep front end: convert -> boolean -> convert back.
session_cpp::BRep boolean(const session_cpp::BRep& A, const session_cpp::BRep& B,
                          BoolOp op, double tol = 1e-6, bool* intersected = nullptr);

} // namespace v3
