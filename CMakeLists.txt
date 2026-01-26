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
    src/knot.cpp
    src/mesh.cpp
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
    add_compile_options(/W4 /utf-8)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

#############################################################################################
# PROTOBUF - BUILD FROM SOURCE (ensures MSVC version compatibility)
#############################################################################################

message(STATUS "Protocol Buffers: ENABLED (building from source)")

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
set(PROTOC_VERSION "33.2")

# Protobuf build options (simplified from examples)
set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(protobuf_MSVC_STATIC_RUNTIME OFF CACHE BOOL "" FORCE)
set(protobuf_WITH_ZLIB OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_PROTOC_BINARIES ON CACHE BOOL "" FORCE)
set(protobuf_BUILD_LIBPROTOC ON CACHE BOOL "" FORCE)
set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "" FORCE)

FetchContent_Declare(protobuf
    GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
    GIT_TAG v${PROTOC_VERSION}
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(protobuf)

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

# Generate proto files
set(GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
file(MAKE_DIRECTORY ${GENERATED_DIR})

if(TARGET protoc)
    set(PROTOC_BIN $<TARGET_FILE:protoc>)
else()
    find_program(PROTOC_BIN protoc)
endif()

file(GLOB PROTO_FILES "${SESSION_PROTO_DIR}/*.proto")
foreach(PROTO_FILE ${PROTO_FILES})
    get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
    list(APPEND PROTO_SOURCES "${GENERATED_DIR}/${PROTO_NAME}.pb.cc")
    list(APPEND PROTO_HEADERS "${GENERATED_DIR}/${PROTO_NAME}.pb.h")
    add_custom_command(
        OUTPUT "${GENERATED_DIR}/${PROTO_NAME}.pb.cc" "${GENERATED_DIR}/${PROTO_NAME}.pb.h"
        COMMAND ${PROTOC_BIN} --cpp_out=${GENERATED_DIR} --proto_path=${SESSION_PROTO_DIR} ${PROTO_FILE}
        DEPENDS ${PROTO_FILE}
        COMMENT "Generating ${PROTO_NAME}.pb.cc/h"
    )
endforeach()

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

target_include_directories(session_core PUBLIC
    ${GENERATED_DIR}
    ${protobuf_SOURCE_DIR}/src
)
target_link_libraries(session_core PUBLIC libprotobuf)
add_dependencies(session_core proto_objects)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE session_core)
LINK_PROTOBUF(${PROJECT_NAME})

#############################################################################################
# TESTS
#############################################################################################
add_library(catch2_lib STATIC src/catch/catch_amalgamated.cpp)
target_include_directories(catch2_lib PUBLIC ${PROJECT_SOURCE_DIR}/src/catch/include)

file(GLOB TEST_SOURCES "src/*_test.cpp")
list(FILTER TEST_SOURCES EXCLUDE REGEX ".*/point_test\\.cpp$")
list(FILTER TEST_SOURCES EXCLUDE REGEX ".*/color_test\\.cpp$")

add_executable(tests ${TEST_SOURCES})
target_include_directories(tests PRIVATE ${PROJECT_SOURCE_DIR}/src/catch/include)
target_link_libraries(tests PRIVATE session_core catch2_lib)
LINK_PROTOBUF(tests)

set(MINITEST_SOURCES
    src/point_test.cpp src/color_test.cpp src/vector_test.cpp src/tolerance_test.cpp
    src/line_test.cpp src/polyline_test.cpp src/plane_test.cpp src/pointcloud_test.cpp
    src/xform_test.cpp src/mesh_test.cpp src/knot_test.cpp src/nurbscurve_test.cpp src/nurbssurface_test.cpp
    src/primitives_test.cpp
)
add_executable(point_minitest src/mini_test.cpp ${MINITEST_SOURCES})
target_link_libraries(point_minitest PRIVATE session_core)
LINK_PROTOBUF(point_minitest)
