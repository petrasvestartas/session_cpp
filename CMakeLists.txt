# Set the minimum version of CMake required
cmake_minimum_required(VERSION 3.22)

# Define the project and the languages used
project(MyProject VERSION 1.0 LANGUAGES CXX)

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Enable compilation database generation for better IntelliSense
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Specify the C++ standard (C++17+ for parallel algorithms)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
message(STATUS "Using C++ standard library parallel execution (std::execution::par)")

# Add compiler warning flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
elseif(MSVC)
    add_compile_options(/W4)
    add_compile_options(/utf-8)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
endif()

#############################################################################################
# PROTOBUF WITH FETCHCONTENT - OPTIONAL
#############################################################################################
# Uses FetchContent for simpler cross-platform builds (no PowerShell/bash differences)
# Control with: cmake -DENABLE_PROTOBUF=ON/OFF ..
# Default: ON
#############################################################################################

option(ENABLE_PROTOBUF "Enable Protocol Buffers support" ON)

if(ENABLE_PROTOBUF)
    message(STATUS "Protocol Buffers: ENABLED")
    add_compile_definitions(ENABLE_PROTOBUF)

    include(FetchContent)
    set(FETCHCONTENT_QUIET OFF)

    # Generated files directory
    set(PROTO_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated")
    file(MAKE_DIRECTORY ${PROTO_GENERATED_DIR})

    # Abseil configuration (must be set before FetchContent_MakeAvailable)
    set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "" FORCE)
    set(ABSL_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
    set(BUILD_TESTING OFF CACHE BOOL "" FORCE)

    # Protobuf configuration
    set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_CONFORMANCE OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    set(protobuf_BUILD_LIBPROTOC ON CACHE BOOL "" FORCE)
    set(protobuf_ABSL_PROVIDER "module" CACHE STRING "" FORCE)
    set(protobuf_WITH_ZLIB OFF CACHE BOOL "" FORCE)
    set(utf8_range_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
    set(utf8_range_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)

    # Fetch Abseil
    FetchContent_Declare(abseil
        GIT_REPOSITORY https://github.com/abseil/abseil-cpp.git
        GIT_TAG 20240722.0
        GIT_SHALLOW TRUE
    )

    # Fetch Protobuf
    FetchContent_Declare(protobuf
        GIT_REPOSITORY https://github.com/protocolbuffers/protobuf.git
        GIT_TAG v29.0
        GIT_SHALLOW TRUE
    )

    # Make both available (protobuf will find abseil automatically via "module" provider)
    FetchContent_MakeAvailable(abseil protobuf)

    # Proto file management and code generation
    set(GLOBAL_PROTO_FILES "" CACHE INTERNAL "")

    function(append_proto_file filename)
        set(GLOBAL_PROTO_FILES ${GLOBAL_PROTO_FILES} ${filename} CACHE INTERNAL "")
    endfunction()

    function(CREATE_CPP_PROTO)
        list(REMOVE_DUPLICATES GLOBAL_PROTO_FILES)

        set(GENERATED_SOURCES "")
        set(GENERATED_HEADERS "")

        foreach(PROTO_FILE ${GLOBAL_PROTO_FILES})
            get_filename_component(PROTO_NAME ${PROTO_FILE} NAME_WE)
            get_filename_component(PROTO_DIR ${PROTO_FILE} DIRECTORY)

            set(GENERATED_CC "${PROTO_GENERATED_DIR}/${PROTO_NAME}.pb.cc")
            set(GENERATED_H "${PROTO_GENERATED_DIR}/${PROTO_NAME}.pb.h")

            list(APPEND GENERATED_SOURCES ${GENERATED_CC})
            list(APPEND GENERATED_HEADERS ${GENERATED_H})

            add_custom_command(
                OUTPUT ${GENERATED_CC} ${GENERATED_H}
                COMMAND protobuf::protoc
                ARGS --cpp_out=${PROTO_GENERATED_DIR} --proto_path=${PROTO_DIR} ${PROTO_FILE}
                DEPENDS protobuf::protoc ${PROTO_FILE}
                COMMENT "Generating C++ code from ${PROTO_FILE}"
                VERBATIM
            )
        endforeach()

        add_library(proto-objects OBJECT ${GENERATED_SOURCES})
        target_include_directories(proto-objects PUBLIC ${PROTO_GENERATED_DIR})
        target_link_libraries(proto-objects PUBLIC protobuf::libprotobuf)

        add_custom_target(PROTO_FILES_READY
            DEPENDS ${GENERATED_SOURCES} ${GENERATED_HEADERS}
            COMMENT "Proto files generation completed"
        )

        set_source_files_properties(${GENERATED_SOURCES} ${GENERATED_HEADERS} PROPERTIES GENERATED TRUE)
    endfunction()

    # Helper function to link protobuf libraries
    function(LINK_PROTOBUF_LIBRARIES target_name)
        target_link_libraries(${target_name} PRIVATE
            $<TARGET_OBJECTS:proto-objects>
            protobuf::libprotobuf
        )
        # macOS needs CoreFoundation for Abseil's time zone code
        if(APPLE)
            find_library(COREFOUNDATION_LIBRARY CoreFoundation REQUIRED)
            target_link_libraries(${target_name} PRIVATE ${COREFOUNDATION_LIBRARY})
        endif()
    endfunction()

    # Register proto files - check both submodule path and sibling path
    file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/session_proto/*.proto")
    if(NOT PROTO_FILES)
        file(GLOB PROTO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../session_proto/*.proto")
    endif()

    foreach(PROTO_FILE ${PROTO_FILES})
        append_proto_file(${PROTO_FILE})
    endforeach()

    # Generate proto files
    CREATE_CPP_PROTO()
else()
    message(STATUS "Protocol Buffers: DISABLED")
endif()

#############################################################################################
# YOUR ORIGINAL PROJECT - Automatically collect all .cpp files in src directory
#############################################################################################

# Collect source files except test files
file(GLOB PROJECT_SOURCES "src/*.cpp")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*_test\\.cpp$")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*_minitest\\.cpp$")

# Create OBJECT library - compiles sources ONCE and reuses object files
add_library(session_core OBJECT ${PROJECT_SOURCES})

# Include directories for the object library
target_include_directories(session_core PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/fmt/include
    ${PROJECT_SOURCE_DIR}/src/json/include
    ${PROJECT_SOURCE_DIR}/src/guid/include
)

# Conditionally add protobuf support to the object library
if(ENABLE_PROTOBUF)
    target_include_directories(session_core PUBLIC ${PROTO_GENERATED_DIR})
    target_link_libraries(session_core PUBLIC protobuf::libprotobuf)
    add_dependencies(session_core PROTO_FILES_READY proto-objects)
endif()

# Add executable for the main project
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE session_core)

if(ENABLE_PROTOBUF)
    LINK_PROTOBUF_LIBRARIES(${PROJECT_NAME})
endif()

#############################################################################################
# Create Catch2 library - optimized static library
#############################################################################################

add_library(catch2_lib STATIC src/catch/catch_amalgamated.cpp)
target_include_directories(catch2_lib PUBLIC ${PROJECT_SOURCE_DIR}/src/catch/include)

# Collect test files
file(GLOB TEST_SOURCES "src/*_test.cpp")

# Exclude protobuf test if protobuf is disabled
if(NOT ENABLE_PROTOBUF)
    list(FILTER TEST_SOURCES EXCLUDE REGEX ".*session_proto_test\\.cpp$")
endif()

# Exclude mini-test files (use mini_test framework instead of Catch2)
list(FILTER TEST_SOURCES EXCLUDE REGEX ".*/point_test\\.cpp$")
list(FILTER TEST_SOURCES EXCLUDE REGEX ".*/color_test\\.cpp$")

# Create test executable
add_executable(tests ${TEST_SOURCES})
target_include_directories(tests PRIVATE ${PROJECT_SOURCE_DIR}/src/catch/include)
target_link_libraries(tests PRIVATE session_core catch2_lib)

if(ENABLE_PROTOBUF)
    LINK_PROTOBUF_LIBRARIES(tests)
endif()

# Mini-test executable
set(MINITEST_SOURCES
    src/point_test.cpp
    src/color_test.cpp
    src/vector_test.cpp
    src/tolerance_test.cpp
    src/line_test.cpp
    src/polyline_test.cpp
    src/plane_test.cpp
    src/pointcloud_test.cpp
    src/xform_test.cpp
    src/mesh_test.cpp
    src/nurbscurve_test.cpp
)
add_executable(point_minitest
    src/mini_test.cpp
    ${MINITEST_SOURCES}
)
target_link_libraries(point_minitest PRIVATE session_core)

if(ENABLE_PROTOBUF)
    LINK_PROTOBUF_LIBRARIES(point_minitest)
endif()

#############################################################################################
# Examples
#############################################################################################
file(GLOB EXAMPLE_DIRS "examples/*")

foreach(EXAMPLE_DIR ${EXAMPLE_DIRS})
    file(GLOB EXAMPLE_FILES "${EXAMPLE_DIR}/*.cpp")

    foreach(EXAMPLE_FILE ${EXAMPLE_FILES})
        get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
        add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
        target_link_libraries(${EXAMPLE_NAME} PRIVATE session_core)

        if(ENABLE_PROTOBUF)
            LINK_PROTOBUF_LIBRARIES(${EXAMPLE_NAME})
        endif()

        get_filename_component(EXAMPLE_DIR_NAME ${EXAMPLE_DIR} NAME)
        set_target_properties(${EXAMPLE_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${EXAMPLE_DIR_NAME})
    endforeach()
endforeach()
