cmake_minimum_required(VERSION 3.20)

project(AuroX
    VERSION 0.1
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Treat all source files as UTF-8 so non-ASCII (e.g. Chinese) content in
# comments/strings parses correctly under MSVC, which otherwise defaults to the
# system codepage and misreads UTF-8 bytes.
if(MSVC)
    add_compile_options(/utf-8)
endif()

# External libraries
set(CPP_HTTPLIB_ROOT "${PROJECT_SOURCE_DIR}/../Library/cpp-httplib")
set(NLOHMANN_JSON_ROOT "${PROJECT_SOURCE_DIR}/../Library/nlohmann/json")

file(GLOB_RECURSE AUROX_SRC
    src/*.cpp
)

add_library(
    AuroX
    STATIC
    ${AUROX_SRC}
)

target_include_directories(
    AuroX
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${NLOHMANN_JSON_ROOT}
)

add_executable(
    AuroX_Exe
    apps/AuroXServer/main.cpp
)

target_include_directories(
    AuroX_Exe
    PRIVATE
        ${CPP_HTTPLIB_ROOT}
        ${NLOHMANN_JSON_ROOT}
)

target_link_libraries(
    AuroX_Exe
    PRIVATE
        AuroX
)
