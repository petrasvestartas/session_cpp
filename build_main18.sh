#!/usr/bin/env bash
# Build main_18 WITHOUT touching CMakeLists.txt (shared with concurrent sessions).
# It reuses the object files cmake already produced for main_16 in the build dir given
# as $1 (default build_cone) and reuses main_16's link line verbatim, with
# main_16.cpp.o swapped for main_18.cpp.o.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BD="${1:-build_cone}"
cd "$ROOT/$BD"
cmake --build . --target main_16 -j "${J:-24}" >/dev/null
FLAGS=$(grep -m1 -- '-o CMakeFiles/main_16.dir/main_16.cpp.o' <(grep -h 'main_16.cpp.o' CMakeFiles/main_16.dir/*.make 2>/dev/null || true) || true)
/usr/bin/c++ -DNDEBUG -O3 -march=native -std=gnu++23 \
    -I"$ROOT" -I"$ROOT/src" -I"$ROOT/src/fmt/include" -I"$ROOT/src/json/include" \
    -I"$ROOT/src/guid/include" -I"$ROOT/src/yaml" -I"$ROOT/src/v2" \
    -isystem "$ROOT/generated" -isystem _deps/protobuf-src/src \
    -c "$ROOT/main_18.cpp" -o CMakeFiles/main_16.dir/main_18.cpp.o
LINK=$(sed 's|CMakeFiles/main_16.dir/main_16.cpp.o|CMakeFiles/main_16.dir/main_18.cpp.o|; s|-o main_16|-o main_18|; s|-Wl,--dependency-file=CMakeFiles/main_16.dir/link.d||' CMakeFiles/main_16.dir/link.txt)
eval "$LINK"
echo "built $BD/main_18"
