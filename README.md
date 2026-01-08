# session_cpp

## Build

```bash
cd session_cpp
cmake -B build
cmake --build build --config Release --parallel
```

## Build Speed Optimizations

Two optimizations are enabled by default: **Unity Build** and **Precompiled Headers (PCH)**.

### Unity Build

Combines multiple `.cpp` files into single compilation units (~59 files → ~6 units). Headers are parsed once per batch instead of once per file.

### Precompiled Headers

Stable STL headers are pre-parsed once and reused. Defined in `src/pch.h`.

### Active Development Workflow

Files under active development should be **excluded from Unity Build** for faster single-file iteration.

| Task | Action |
|------|--------|
| Start developing a class | Add `.cpp` to `UNITY_BUILD_EXCLUDE` in CMakeLists.txt |
| Finish developing a class | Remove from `UNITY_BUILD_EXCLUDE` |
| Modify `src/pch.h` | Delete `build/CMakeFiles` to force PCH rebuild |

**Currently excluded (active development):**
- `nurbscurve.cpp`
- `nurbssurface.cpp`
- `knot.cpp`
- `mesh.cpp`

### Disable Optimizations

```bash
cmake -B build -DENABLE_UNITY_BUILD=OFF -DENABLE_PCH=OFF
```

### PCH Guidelines

- Only add stable STL/external headers to `src/pch.h`
- Do NOT add project headers under active development
- Do NOT add headers that change frequently
