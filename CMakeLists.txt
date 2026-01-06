cmake_minimum_required(VERSION 3.22)
project(MyProject VERSION 1.0 LANGUAGES CXX)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
elseif(MSVC)
    add_compile_options(/W4 /utf-8)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

#############################################################################################
# PROTOBUF - OPTIONAL (uses pre-built session_proto release)
#############################################################################################
option(ENABLE_PROTOBUF "Enable Protocol Buffers support" ON)

if(ENABLE_PROTOBUF)
    message(STATUS "Protocol Buffers: ENABLED")
    add_compile_definitions(ENABLE_PROTOBUF)

    include(FetchContent)
    set(FETCHCONTENT_QUIET OFF)

    set(DEPS_DIR "${CMAKE_BINARY_DIR}/deps")
    set(PROTO_DIR "${DEPS_DIR}/session_proto")
    file(MAKE_DIRECTORY ${DEPS_DIR})

    set(SESSION_PROTO_VERSION "v0.0.14")

    # Platform-specific release
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(PROTO_ARTIFACT "session-proto-linux-x64")
        set(PROTO_EXT "tar.gz")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
            set(PROTO_ARTIFACT "session-proto-macos-arm64")
        else()
            set(PROTO_ARTIFACT "session-proto-macos-x64")
        endif()
        set(PROTO_EXT "tar.gz")
    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
        set(PROTO_ARTIFACT "session-proto-windows-x64")
        set(PROTO_EXT "zip")
    endif()

    if(NOT EXISTS "${PROTO_DIR}/lib")
        message(STATUS "Downloading session_proto ${SESSION_PROTO_VERSION}...")
        FetchContent_Declare(session_proto
            URL https://github.com/petrasvestartas/session_proto/releases/download/${SESSION_PROTO_VERSION}/${PROTO_ARTIFACT}.${PROTO_EXT}
            SOURCE_DIR ${PROTO_DIR}
        )
        FetchContent_Populate(session_proto)
    else()
        message(STATUS "session_proto already present")
    endif()

    # Import pre-built libraries
    add_library(protobuf_lib STATIC IMPORTED)

    if(WIN32)
        set_target_properties(protobuf_lib PROPERTIES IMPORTED_LOCATION "${PROTO_DIR}/lib/libprotobuf.lib")
        file(GLOB ABSL_LIBS "${PROTO_DIR}/lib/absl_*.lib")
        # utf8_validity may be statically linked into protobuf or in subdirectory
        if(EXISTS "${PROTO_DIR}/lib/utf8_validity.lib")
            add_library(utf8_lib STATIC IMPORTED)
            set_target_properties(utf8_lib PROPERTIES IMPORTED_LOCATION "${PROTO_DIR}/lib/utf8_validity.lib")
            set(HAS_UTF8_LIB TRUE)
        else()
            set(HAS_UTF8_LIB FALSE)
        endif()
    else()
        set_target_properties(protobuf_lib PROPERTIES IMPORTED_LOCATION "${PROTO_DIR}/lib/libprotobuf.a")
        file(GLOB ABSL_LIBS "${PROTO_DIR}/lib/libabsl_*.a")
        if(EXISTS "${PROTO_DIR}/lib/libutf8_validity.a")
            add_library(utf8_lib STATIC IMPORTED)
            set_target_properties(utf8_lib PROPERTIES IMPORTED_LOCATION "${PROTO_DIR}/lib/libutf8_validity.a")
            set(HAS_UTF8_LIB TRUE)
        else()
            set(HAS_UTF8_LIB FALSE)
        endif()
    endif()

    # Compile generated proto sources
    file(GLOB PROTO_SOURCES "${PROTO_DIR}/src/*.pb.cc")
    add_library(proto_objects OBJECT ${PROTO_SOURCES})
    target_include_directories(proto_objects PUBLIC
        ${PROTO_DIR}/include/generated
        ${PROTO_DIR}/include
        ${PROTO_DIR}/include/utf8_range
    )

    # Helper to link protobuf
    function(LINK_PROTOBUF target_name)
        target_include_directories(${target_name} PRIVATE
            ${PROTO_DIR}/include/generated
            ${PROTO_DIR}/include
            ${PROTO_DIR}/include/utf8_range
        )
        if(UNIX AND NOT APPLE)
            # Linux: use linker group to resolve circular dependencies in static libs
            if(HAS_UTF8_LIB)
                target_link_libraries(${target_name} PRIVATE
                    $<TARGET_OBJECTS:proto_objects>
                    -Wl,--start-group
                    protobuf_lib ${ABSL_LIBS} utf8_lib
                    -Wl,--end-group
                    pthread
                )
            else()
                target_link_libraries(${target_name} PRIVATE
                    $<TARGET_OBJECTS:proto_objects>
                    -Wl,--start-group
                    protobuf_lib ${ABSL_LIBS}
                    -Wl,--end-group
                    pthread
                )
            endif()
        else()
            if(HAS_UTF8_LIB)
                target_link_libraries(${target_name} PRIVATE
                    $<TARGET_OBJECTS:proto_objects>
                    protobuf_lib ${ABSL_LIBS} utf8_lib
                )
            else()
                target_link_libraries(${target_name} PRIVATE
                    $<TARGET_OBJECTS:proto_objects>
                    protobuf_lib ${ABSL_LIBS}
                )
            endif()
        endif()
        if(APPLE)
            find_library(COREFOUNDATION CoreFoundation REQUIRED)
            target_link_libraries(${target_name} PRIVATE ${COREFOUNDATION})
        endif()
    endfunction()
else()
    message(STATUS "Protocol Buffers: DISABLED")
endif()

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

if(ENABLE_PROTOBUF)
    target_include_directories(session_core PUBLIC
        ${PROTO_DIR}/include/generated
        ${PROTO_DIR}/include
        ${PROTO_DIR}/include/utf8_range
    )
    add_dependencies(session_core proto_objects)
endif()

# Main executable
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE session_core)
if(ENABLE_PROTOBUF)
    LINK_PROTOBUF(${PROJECT_NAME})
endif()

#############################################################################################
# TESTS
#############################################################################################
add_library(catch2_lib STATIC src/catch/catch_amalgamated.cpp)
target_include_directories(catch2_lib PUBLIC ${PROJECT_SOURCE_DIR}/src/catch/include)

file(GLOB TEST_SOURCES "src/*_test.cpp")
if(NOT ENABLE_PROTOBUF)
    list(FILTER TEST_SOURCES EXCLUDE REGEX ".*session_proto_test\\.cpp$")
endif()
list(FILTER TEST_SOURCES EXCLUDE REGEX ".*/point_test\\.cpp$")
list(FILTER TEST_SOURCES EXCLUDE REGEX ".*/color_test\\.cpp$")

add_executable(tests ${TEST_SOURCES})
target_include_directories(tests PRIVATE ${PROJECT_SOURCE_DIR}/src/catch/include)
target_link_libraries(tests PRIVATE session_core catch2_lib)
if(ENABLE_PROTOBUF)
    LINK_PROTOBUF(tests)
endif()

# Mini-tests
set(MINITEST_SOURCES
    src/point_test.cpp src/color_test.cpp src/vector_test.cpp src/tolerance_test.cpp
    src/line_test.cpp src/polyline_test.cpp src/plane_test.cpp src/pointcloud_test.cpp
    src/xform_test.cpp src/mesh_test.cpp src/nurbscurve_test.cpp
)
add_executable(point_minitest src/mini_test.cpp ${MINITEST_SOURCES})
target_link_libraries(point_minitest PRIVATE session_core)
if(ENABLE_PROTOBUF)
    LINK_PROTOBUF(point_minitest)
endif()

#############################################################################################
# EXAMPLES
#############################################################################################
file(GLOB EXAMPLE_DIRS "examples/*")
foreach(EXAMPLE_DIR ${EXAMPLE_DIRS})
    file(GLOB EXAMPLE_FILES "${EXAMPLE_DIR}/*.cpp")
    foreach(EXAMPLE_FILE ${EXAMPLE_FILES})
        get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
        add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
        target_link_libraries(${EXAMPLE_NAME} PRIVATE session_core)
        if(ENABLE_PROTOBUF)
            LINK_PROTOBUF(${EXAMPLE_NAME})
        endif()
        get_filename_component(EXAMPLE_DIR_NAME ${EXAMPLE_DIR} NAME)
        set_target_properties(${EXAMPLE_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples/${EXAMPLE_DIR_NAME})
    endforeach()
endforeach()
