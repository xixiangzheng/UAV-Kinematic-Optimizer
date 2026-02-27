cmake_minimum_required(VERSION 3.10)
project(SmokeScreenOptimization LANGUAGES CXX)

# --- Compilation Settings ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable O3 optimization for high-dimensional differential evolution performance
if(MSVC)
    add_compile_options(/W4 /O2)
else()
    add_compile_options(-O3 -Wall -Wextra)
endif()

# --- Path Management ---
# Output all executables to the root bin/ directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

# Global include path for configuration headers
include_directories(${CMAKE_SOURCE_DIR}/config)

# --- Target Automation ---
# Automatically scan all source files in the core/ directory
file(GLOB SOURCES "core/*.cpp")

foreach(sourcefile ${SOURCES})
    get_filename_component(name ${sourcefile} NAME_WE)
    
    add_executable(${name} ${sourcefile})
    
    message(STATUS "Configuring target: ${name}")
endforeach()