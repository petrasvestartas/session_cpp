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

option(ENABLE_UNITY_BUILD "Enable unity build for faster compilation" ON)
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
    src/graph.cpp
    src/tree.cpp
    src/session.cpp
    src/element.cpp
)

#############################################################################################
# COMPILER CACHE (sccache/ccache)
#############################################################################################
find_program(SCCACHE_PROGRAM sccache)
find_program(CCACHE_PROGRAM ccache)
if(SCCACHE_PROGRAM)
    message(STATUS "Using sccache: ${SCCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${SCCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${SCCACHE_PROGRAM}")
elseif(CCACHE_PROGRAM)
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
elseif(MSVC)
    add_compile_options(/W4 /utf-8 /external:W0)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

#############################################################################################
# PROTOBUF - BUILD FROM SOURCE (ensures MSVC version compatibility)
#############################################################################################

message(STATUS "Protocol Buffers: ENABLED (building from source)")

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(PROTOC_VERSION "33.2")

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

FetchContent_Declare(protobuf
    GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
    GIT_TAG v${PROTOC_VERSION}
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(protobuf)

#############################################################################################
# CLIPPER2
#############################################################################################
FetchContent_Declare(Clipper2
    GIT_REPOSITORY https://github.com/AngusJohnson/Clipper2.git
    GIT_TAG Clipper2_1.4.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR CPP
)
set(CLIPPER2_UTILS OFF CACHE BOOL "" FORCE)
set(CLIPPER2_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CLIPPER2_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Clipper2)
if(MSVC)
    target_compile_options(Clipper2 PRIVATE /w)
endif()

# Suppress warnings from protobuf and absl third-party libraries
if(MSVC)
    target_compile_options(libprotobuf PRIVATE /w)
    target_compile_options(libprotobuf-lite PRIVATE /w)
    if(TARGET utf8_validity)
        target_compile_options(utf8_validity PRIVATE /w)
    endif()
    if(TARGET utf8_range)
        target_compile_options(utf8_range PRIVATE /w)
    endif()
    if(TARGET libupb)
        target_compile_options(libupb PRIVATE /w)
    endif()
    get_property(ABSL_TARGETS DIRECTORY "${absl_SOURCE_DIR}" PROPERTY BUILDSYSTEM_TARGETS)
    foreach(TGT IN LISTS ABSL_TARGETS)
        get_target_property(TGT_TYPE ${TGT} TYPE)
        if(TGT_TYPE STREQUAL "STATIC_LIBRARY" OR TGT_TYPE STREQUAL "OBJECT_LIBRARY")
            target_compile_options(${TGT} PRIVATE /w)
        endif()
    endforeach()
endif()


# Session proto directory
set(SESSION_PROTO_DIR "${CMAKE_SOURCE_DIR}/../session_proto")
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
set(GENERATED_DIR "${CMAKE_SOURCE_DIR}/generated")

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
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
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

add_library(session_core OBJECT ${PROJECT_SOURCES})
target_include_directories(session_core PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/fmt/include
    ${PROJECT_SOURCE_DIR}/src/json/include
    ${PROJECT_SOURCE_DIR}/src/guid/include
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
    ${protobuf_SOURCE_DIR}/src
)
target_link_libraries(session_core PUBLIC libprotobuf)
add_dependencies(session_core proto_objects)

foreach(MAIN_ID 1 3 4 6)
    add_executable(main_${MAIN_ID} main_${MAIN_ID}.cpp)
    target_link_libraries(main_${MAIN_ID} PRIVATE session_core)
    LINK_PROTOBUF(main_${MAIN_ID})
    if(ENABLE_PCH)
        target_precompile_headers(main_${MAIN_ID} PRIVATE ${PROJECT_SOURCE_DIR}/src/pch.h)
    endif()
endforeach()
# main_2 uses wood_session::plane_to_face for cross-joint detection,
# so it needs wood_joint_detection.cpp (defines wood_session::CrossJoint + plane_to_face).
add_executable(main_2 main_2.cpp wood/wood_joint_detection.cpp)
target_include_directories(main_2 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(main_2 PRIVATE session_core)
LINK_PROTOBUF(main_2)
if(ENABLE_PCH)
    target_precompile_headers(main_2 PRIVATE ${PROJECT_SOURCE_DIR}/src/pch.h)
endif()
# main_5 (wood face_to_face port — uses native session boolean, no Clipper2)
# Split into wood/ subdirectory mirroring wood's cmake/src/wood/include/ tree.
add_executable(main_5
    main_5.cpp
    wood/wood_beams.cpp
    wood/wood_element.cpp
    wood/wood_face_to_face.cpp
    wood/wood_globals.cpp
    wood/wood_internal.cpp
    wood/wood_joint.cpp
    wood/wood_joint_detection.cpp
    wood/wood_main.cpp
    wood/wood_merge.cpp
    wood/wood_test.cpp
)
target_include_directories(main_5 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(main_5 PRIVATE session_core)
LINK_PROTOBUF(main_5)
if(ENABLE_PCH)
    target_precompile_headers(main_5 PRIVATE ${PROJECT_SOURCE_DIR}/src/pch.h)
endif()

# main_wood_* tutorial files (see wood/README.md for the concepts)
set(WOOD_SOURCES
    wood/wood_beams.cpp
    wood/wood_element.cpp
    wood/wood_face_to_face.cpp
    wood/wood_globals.cpp
    wood/wood_internal.cpp
    wood/wood_joint.cpp
    wood/wood_joint_detection.cpp
    wood/wood_main.cpp
    wood/wood_merge.cpp
)
function(ADD_WOOD_EXAMPLE NAME)
    add_executable(${NAME} wood/${NAME}.cpp ${WOOD_SOURCES} ${ARGN})
    target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${NAME} PRIVATE session_core)
    LINK_PROTOBUF(${NAME})
    if(ENABLE_PCH)
        target_precompile_headers(${NAME} PRIVATE ${PROJECT_SOURCE_DIR}/src/pch.h)
    endif()
endfunction()
ADD_WOOD_EXAMPLE(main_wood_00_hello)
ADD_WOOD_EXAMPLE(main_wood_01_new_geometry)
ADD_WOOD_EXAMPLE(main_wood_02_joint_types)
ADD_WOOD_EXAMPLE(main_wood_03_new_joint)
ADD_WOOD_EXAMPLE(main_wood_04_all_datasets wood/wood_test.cpp)

#############################################################################################
# TESTS
#############################################################################################
set(MINITEST_SOURCES
    src/point_test.cpp src/color_test.cpp src/vector_test.cpp src/tolerance_test.cpp src/session_config_test.cpp
    src/line_test.cpp src/polyline_test.cpp src/plane_test.cpp src/pointcloud_test.cpp
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
    src/file_step_test.cpp
    src/objects_test.cpp
    src/quaternion_test.cpp
    src/session_test.cpp
    src/tree_test.cpp
    src/brep_test.cpp
    src/element_test.cpp
    src/element_beam_test.cpp
    src/element_column_test.cpp
    src/element_plate_test.cpp
    src/matrix_test.cpp
    src/convex_hull_test.cpp
    src/spatial_kdtree_test.cpp
    src/boolean_polyline_test.cpp
    src/elementfeature_test.cpp
)
add_executable(point_minitest src/mini_test.cpp ${MINITEST_SOURCES})
target_link_libraries(point_minitest PRIVATE session_core)
LINK_PROTOBUF(point_minitest)

