cmake_minimum_required(VERSION 3.22)
project(MyProject VERSION 1.0 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

#############################################################################################
# BUILD SPEED OPTIMIZATIONS (Unity Build + PCH)
#############################################################################################
# Unity Build: Combines multiple .cpp files into single compilation units.
# - Reduces 59 compilations to ~6, headers parsed once per batch
# - Faster for header changes, slightly slower for single .cpp changes
#
# Precompiled Headers (PCH): Pre-parses stable STL headers once.
# - Defined in src/pch.h, contains only STL headers
# - Speeds up incremental builds by 20-40%
#
# WORKFLOW FOR ACTIVE DEVELOPMENT:
# --------------------------------
# Files under active development should be EXCLUDED from Unity Build.
# This allows faster incremental builds when editing those specific files.
#
# To exclude a file from Unity Build (for active development):
#   set_source_files_properties(src/myclass.cpp PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
#
# To re-include a file (when development is complete):
#   Remove the set_source_files_properties line for that file
#
# ADDING NEW CLASSES:
# -------------------
# 1. During active development: Add to UNITY_BUILD_EXCLUDE list below
# 2. When stable: Remove from UNITY_BUILD_EXCLUDE list
#
# PCH NOTES:
# ----------
# - Only add stable STL/external headers to src/pch.h
# - Do NOT add project headers under active development
# - After modifying pch.h, delete build/CMakeFiles to force rebuild
#############################################################################################

# Unity on in CI (fewer compiles from scratch), off locally: an edited .cpp then recompiles alone (6 s, not its 10-file batch at 16 s).
if(DEFINED ENV{CI})
    option(ENABLE_UNITY_BUILD "Enable unity build for faster compilation" ON)
else()
    option(ENABLE_UNITY_BUILD "Enable unity build for faster compilation" OFF)
endif()
option(ENABLE_PCH "Enable precompiled headers" ON)

# Files under active development - excluded from Unity Build for faster iteration
# Move files here when actively developing, remove when stable
set(UNITY_BUILD_EXCLUDE
    src/nurbscurve.cpp
    src/nurbssurface.cpp
    src/nurbssurface_trimmed.cpp
    src/nurbsknot.cpp
    src/mesh.cpp
    src/brep.cpp
    src/intersection.cpp
    src/graph.cpp
    src/tree.cpp
    src/session.cpp
    src/element.cpp
    src/yaml/yaml.cpp
    src/mesh_offset.cpp
    src/io_xyz.cpp
)

#############################################################################################
# COMPILER CACHE (sccache/ccache)
#############################################################################################
find_program(SCCACHE_PROGRAM sccache)
find_program(CCACHE_PROGRAM ccache)
if(SCCACHE_PROGRAM AND NOT MSVC)
    message(STATUS "Using sccache: ${SCCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${SCCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${SCCACHE_PROGRAM}")
elseif(CCACHE_PROGRAM AND NOT MSVC)
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(SESSION_COMPILE_OPTIONS -Wall -Wextra -Wpedantic)
    # -march=native bakes the BUILD machine's instruction set into the objects. That is fine on one
    # developer machine and unsafe anywhere the artefacts can move: CI caches session_cpp/build/,
    # and GitHub's ubuntu-22.04 fleet is a mix of CPU models, so objects compiled where AVX-512
    # exists get reused where it does not. The binary then dies on its first instruction -
    # "Illegal instruction (core dumped)", exit 132 - after a build that looked completely healthy,
    # which makes it read as a test failure rather than a toolchain one.
    #
    # Default off, so a build is portable to any machine that can run the compiler's baseline.
    # Turn it on for local number-crunching: cmake -DSESSION_NATIVE_ARCH=ON
    # (A portable middle ground, if the baseline ever proves too slow, is -march=x86-64-v2: SSE4.2
    # and POPCNT, present on anything since ~2009, and safe across every runner.)
    option(SESSION_NATIVE_ARCH
           "Optimise for THIS machine's CPU (-march=native). Local builds only." OFF)
    if(SESSION_NATIVE_ARCH)
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
    else()
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
    endif()
elseif(MSVC)
    set(SESSION_COMPILE_OPTIONS /W4 /utf-8 /external:W0)
endif()

#############################################################################################
# PROTOBUF - BUILD FROM SOURCE (ensures MSVC version compatibility)
#############################################################################################

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(PROTOC_VERSION "36.2")

# Option C: generated .pb.cc/.pb.h are committed under generated/. The runtime
# (libprotobuf) is still required to link against them, but protoc + libprotoc
# are NOT built — saves substantial CMake configure + compile time.
#
# To regenerate after editing ../session_proto/*.proto, run:
#   cmake --build build --target regen_proto
# (the target is defined only when SESSION_REGEN_PROTO=ON, which also
#  re-enables protoc compilation).
option(SESSION_REGEN_PROTO "Build protoc and add a regen_proto target" OFF)

set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(protobuf_MSVC_STATIC_RUNTIME OFF CACHE BOOL "" FORCE)
set(protobuf_WITH_ZLIB OFF CACHE BOOL "" FORCE)
if(SESSION_REGEN_PROTO)
    set(protobuf_BUILD_PROTOC_BINARIES ON CACHE BOOL "" FORCE)
    set(protobuf_BUILD_LIBPROTOC ON CACHE BOOL "" FORCE)
else()
    set(protobuf_BUILD_PROTOC_BINARIES OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_LIBPROTOC OFF CACHE BOOL "" FORCE)
endif()
set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "" FORCE)

# Prebuilt protobuf + abseil from session/bash/deps_protobuf.sh: no clone and no third-party compile per build dir.
# SESSION_DEPS_PREFIX (cache or env) names an install, else ~/.cache/session-deps/protobuf-<version> when present,
# else (CI, a fresh machine) FetchContent as before; outside CI it reuses the sources that script cloned.
set(SESSION_DEPS_PREFIX "$ENV{SESSION_DEPS_PREFIX}" CACHE PATH "Prebuilt protobuf+abseil prefix (session/bash/deps_protobuf.sh)")
if(DEFINED ENV{SESSION_DEPS_ROOT})
    set(SESSION_DEPS_ROOT "$ENV{SESSION_DEPS_ROOT}")
else()
    set(SESSION_DEPS_ROOT "$ENV{HOME}/.cache/session-deps")
endif()
set(SESSION_PROTOBUF_PREFIX "${SESSION_DEPS_PREFIX}")
# CI never picks up a local prefix; global imported targets (wood links them from its own scope) need CMake 3.24
if(NOT SESSION_PROTOBUF_PREFIX AND NOT DEFINED ENV{CI} AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.24
   AND EXISTS "${SESSION_DEPS_ROOT}/protobuf-${PROTOC_VERSION}/.session-deps-ok")
    set(SESSION_PROTOBUF_PREFIX "${SESSION_DEPS_ROOT}/protobuf-${PROTOC_VERSION}")
endif()

if(SESSION_PROTOBUF_PREFIX AND CMAKE_VERSION VERSION_LESS 3.24)
    message(FATAL_ERROR "SESSION_DEPS_PREFIX needs CMake 3.24 or newer (global imported targets)")
endif()

if(SESSION_PROTOBUF_PREFIX)
    message(STATUS "Protocol Buffers: prebuilt at ${SESSION_PROTOBUF_PREFIX}")
    list(PREPEND CMAKE_PREFIX_PATH "${SESSION_PROTOBUF_PREFIX}")
    set(CMAKE_FIND_PACKAGE_TARGETS_GLOBAL ON)
    find_package(Protobuf CONFIG REQUIRED)
    set(CMAKE_FIND_PACKAGE_TARGETS_GLOBAL OFF)
    if(NOT Protobuf_VERSION VERSION_EQUAL PROTOC_VERSION)
        message(FATAL_ERROR "${SESSION_PROTOBUF_PREFIX} holds protobuf ${Protobuf_VERSION}, this tree pins ${PROTOC_VERSION}")
    endif()
    add_library(libprotobuf ALIAS protobuf::libprotobuf)
    add_executable(protoc ALIAS protobuf::protoc)
else()
    message(STATUS "Protocol Buffers: building from source")
    if(NOT DEFINED ENV{CI} AND EXISTS "${SESSION_DEPS_ROOT}/src/protobuf-${PROTOC_VERSION}/.session-deps-clone-ok")
        set(FETCHCONTENT_SOURCE_DIR_PROTOBUF "${SESSION_DEPS_ROOT}/src/protobuf-${PROTOC_VERSION}")
        file(STRINGS "${FETCHCONTENT_SOURCE_DIR_PROTOBUF}/cmake/dependencies.cmake" ABSL_VERSION REGEX "abseil-cpp-version")
        string(REGEX MATCH "[0-9]+\\.[0-9]+" ABSL_VERSION "${ABSL_VERSION}")
        if(EXISTS "${SESSION_DEPS_ROOT}/src/abseil-cpp-${ABSL_VERSION}/.session-deps-clone-ok")
            set(FETCHCONTENT_SOURCE_DIR_ABSL "${SESSION_DEPS_ROOT}/src/abseil-cpp-${ABSL_VERSION}")
        endif()
    endif()
    FetchContent_Declare(protobuf
        GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
        GIT_TAG v${PROTOC_VERSION}
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(protobuf)
endif()

# Suppress warnings from protobuf and absl third-party libraries
if(MSVC)
    set(THIRD_PARTY_COMPILE_OPTIONS /w)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(THIRD_PARTY_COMPILE_OPTIONS -w)
endif()
if(THIRD_PARTY_COMPILE_OPTIONS AND NOT SESSION_PROTOBUF_PREFIX)
    foreach(TGT IN ITEMS libprotobuf libprotobuf-lite utf8_validity utf8_range libupb)
        if(TARGET ${TGT})
            target_compile_options(${TGT} PRIVATE ${THIRD_PARTY_COMPILE_OPTIONS})
        endif()
    endforeach()
    get_property(ABSL_TARGETS DIRECTORY "${absl_SOURCE_DIR}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(TGT IN LISTS ABSL_TARGETS)
        get_target_property(TGT_TYPE ${TGT} TYPE)
        if(TGT_TYPE STREQUAL "STATIC_LIBRARY" OR TGT_TYPE STREQUAL "OBJECT_LIBRARY")
            target_compile_options(${TGT} PRIVATE ${THIRD_PARTY_COMPILE_OPTIONS})
        endif()
    endforeach()
endif()


# Where session_cpp's OWN files live.
#
# CMAKE_CURRENT_LIST_DIR, never CMAKE_SOURCE_DIR. CMAKE_SOURCE_DIR is the TOP-LEVEL
# project's directory - whoever ran cmake - so it only happens to be right when
# session_cpp is the top-level project. Under add_subdirectory() from a consumer it
# becomes the CONSUMER's root, and session_cpp then hunts for its own generated
# protobufs inside that consumer.
#
# That is why wood and wood_nano each mirrored session_cpp/generated into their own
# root: a workaround for this line, not a requirement of theirs. The mirror is also
# what made a schema change corrupt the heap - file(COPY) preserves timestamps, so a
# refreshed element.pb.h could be older than the objects built against the previous
# one, and the build then linked two different layouts of session_proto::Element
# together. With the paths anchored here, no consumer needs a copy at all.
set(SESSION_CPP_ROOT "${CMAKE_CURRENT_LIST_DIR}")

# Session proto directory
set(SESSION_PROTO_DIR "${SESSION_CPP_ROOT}/../session_proto")
if(NOT EXISTS "${SESSION_PROTO_DIR}")
    FetchContent_Declare(session_proto_files
        GIT_REPOSITORY https://github.com/petrasvestartas/session_proto.git
        GIT_TAG main
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(session_proto_files)
    set(SESSION_PROTO_DIR "${session_proto_files_SOURCE_DIR}")
endif()

# Option C: consume committed .pb.cc/.pb.h from session_cpp/generated/.
# No protoc invocation at build time.
set(GENERATED_DIR "${SESSION_CPP_ROOT}/generated")

file(GLOB PROTO_SOURCES "${GENERATED_DIR}/*.pb.cc")
file(GLOB PROTO_HEADERS "${GENERATED_DIR}/*.pb.h")

if(NOT PROTO_SOURCES)
    message(FATAL_ERROR
        "No generated protobuf files found in ${GENERATED_DIR}. "
        "Run `cmake -B build -DSESSION_REGEN_PROTO=ON && cmake --build build --target regen_proto` "
        "to regenerate from ${SESSION_PROTO_DIR}.")
endif()

# Optional regen target — only wired when protoc was actually built.
if(SESSION_REGEN_PROTO AND TARGET protoc)
    file(GLOB PROTO_FILES "${SESSION_PROTO_DIR}/*.proto")
    add_custom_target(regen_proto
        COMMAND ${CMAKE_COMMAND} -E make_directory "${GENERATED_DIR}"
        COMMAND $<TARGET_FILE:protoc>
                --cpp_out=${GENERATED_DIR}
                --proto_path=${SESSION_PROTO_DIR}
                ${PROTO_FILES}
        DEPENDS protoc ${PROTO_FILES}
        WORKING_DIRECTORY "${SESSION_CPP_ROOT}"
        COMMENT "Regenerating committed .pb.cc/.pb.h into generated/"
        VERBATIM
    )
endif()

# Proto library with suppressed warnings
add_library(proto_objects OBJECT ${PROTO_SOURCES})
target_include_directories(proto_objects PUBLIC ${GENERATED_DIR})
target_link_libraries(proto_objects PUBLIC libprotobuf)
if(MSVC)
    target_compile_options(proto_objects PRIVATE /W1 /wd4005 /wd4996 /wd4244 /wd4267 /wd4100 /wd4127)
else()
    target_compile_options(proto_objects PRIVATE -w)
endif()

function(LINK_PROTOBUF target_name)
    target_include_directories(${target_name} PRIVATE ${GENERATED_DIR})
    target_link_libraries(${target_name} PRIVATE $<TARGET_OBJECTS:proto_objects> libprotobuf)
    if(UNIX AND NOT APPLE)
        target_link_libraries(${target_name} PRIVATE pthread)
    endif()
    if(APPLE)
        find_library(COREFOUNDATION CoreFoundation REQUIRED)
        target_link_libraries(${target_name} PRIVATE ${COREFOUNDATION})
    endif()
endfunction()


#############################################################################################
# PROJECT SOURCES
#############################################################################################
file(GLOB PROJECT_SOURCES "src/*.cpp")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*_test\\.cpp$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*_minitest\\.cpp$")
# tiny-yaml lives in src/yaml/ — non-recursive glob misses it.
list(APPEND PROJECT_SOURCES "${PROJECT_SOURCE_DIR}/src/yaml/yaml.cpp")

add_library(session_core OBJECT ${PROJECT_SOURCES})
target_compile_options(session_core PRIVATE ${SESSION_COMPILE_OPTIONS})
if(MSVC)
    target_compile_definitions(session_core PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
target_include_directories(session_core PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/fmt/include
    ${PROJECT_SOURCE_DIR}/src/json/include
    ${PROJECT_SOURCE_DIR}/src/guid/include
    ${PROJECT_SOURCE_DIR}/src/yaml
)

# Unity Build - applied only to session_core (not dependencies)
if(ENABLE_UNITY_BUILD)
    set_target_properties(session_core PROPERTIES UNITY_BUILD ON)
    set_target_properties(session_core PROPERTIES UNITY_BUILD_BATCH_SIZE 10)
    # Exclude actively developed files from Unity Build
    foreach(EXCLUDED_FILE ${UNITY_BUILD_EXCLUDE})
        set_source_files_properties(${EXCLUDED_FILE} PROPERTIES SKIP_UNITY_BUILD_INCLUSION ON)
    endforeach()
endif()

# Precompiled headers - stable STL headers only
if(ENABLE_PCH)
    target_precompile_headers(session_core PRIVATE ${PROJECT_SOURCE_DIR}/src/pch.h)
endif()

target_include_directories(session_core SYSTEM PUBLIC
    ${GENERATED_DIR}
    $<$<BOOL:${protobuf_SOURCE_DIR}>:${protobuf_SOURCE_DIR}/src>
)
target_link_libraries(session_core PUBLIC libprotobuf)
add_dependencies(session_core proto_objects)

# main_3 — session/quaternion/plane demo driver.
add_executable(main_3 main_3.cpp)
target_compile_options(main_3 PRIVATE ${SESSION_COMPILE_OPTIONS})
target_link_libraries(main_3 PRIVATE session_core)
LINK_PROTOBUF(main_3)
if(ENABLE_PCH)
    target_precompile_headers(main_3 PRIVATE ${PROJECT_SOURCE_DIR}/src/pch.h)
endif()

# session_bench — undo/redo cost of the tombstone kernel, printed, never asserted.
add_executable(session_bench examples/bench.cpp)
target_compile_options(session_bench PRIVATE ${SESSION_COMPILE_OPTIONS})
target_link_libraries(session_bench PRIVATE session_core)
LINK_PROTOBUF(session_bench)

#############################################################################################
# TESTS
#############################################################################################
set(MINITEST_SOURCES
    src/point_test.cpp src/color_test.cpp src/vector_test.cpp src/tolerance_test.cpp src/session_config_test.cpp
    src/line_test.cpp src/polyline_test.cpp src/plane_test.cpp src/pointcloud_test.cpp
    src/simple_split_test.cpp
    src/xform_test.cpp src/mesh_test.cpp src/nurbsknot_test.cpp src/nurbscurve_test.cpp src/nurbssurface_test.cpp
    src/closest_test.cpp
    src/primitives_test.cpp
    src/nurbssurface_trimmed_test.cpp
    src/remesh_cdt_test.cpp
    src/remesh_nurbssurface_adaptive_test.cpp
    src/remesh_nurbssurface_grid_test.cpp
    src/obb_test.cpp
    src/spatial_bvh_test.cpp
    src/spatial_rtree_test.cpp
    src/aabb_test.cpp
    src/spatial_aabbtree_test.cpp
    src/file_encoders_test.cpp
    src/graph_test.cpp
    src/intersection_test.cpp
    src/file_obj_test.cpp
    src/io_xyz_test.cpp
    src/file_step_test.cpp
    src/objects_test.cpp
    src/collection_test.cpp
    src/quaternion_test.cpp
    src/session_test.cpp
    src/history_test.cpp
    src/tree_test.cpp
    src/brep_test.cpp
    src/element_test.cpp
    src/matrix_test.cpp
    src/convex_hull_test.cpp
    src/spatial_kdtree_test.cpp
    src/spatial_octree_test.cpp
    src/boolean_polyline_test.cpp
    # src/elementfeature_test.cpp  # removed: source file does not exist (stale reference)
    src/mesh_offset_test.cpp
    src/instance_ref_test.cpp
    src/interaction_test.cpp
)
add_executable(point_minitest src/mini_test.cpp ${MINITEST_SOURCES})
target_compile_options(point_minitest PRIVATE ${SESSION_COMPILE_OPTIONS})
target_link_libraries(point_minitest PRIVATE session_core)
LINK_PROTOBUF(point_minitest)
if(ENABLE_PCH)
    target_precompile_headers(point_minitest REUSE_FROM session_core)
endif()

# Optional local scratch driver (untracked): drop a main_dbg.cpp next to this file to get a
# quick executable linked against the kernel without touching the test targets.
if(EXISTS "${PROJECT_SOURCE_DIR}/main_dbg.cpp")
    add_executable(main_dbg main_dbg.cpp)
    target_compile_options(main_dbg PRIVATE ${SESSION_COMPILE_OPTIONS})
    target_link_libraries(main_dbg PRIVATE session_core)
    LINK_PROTOBUF(main_dbg)
endif()
