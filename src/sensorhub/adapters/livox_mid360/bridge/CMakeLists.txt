
cmake_minimum_required(VERSION 3.10)
project(livox_bridge LANGUAGES CXX)

# --- C++ standard ---
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Options ---
option(BRIDGE_ENABLE_STDOUT "Print NDJSON to stdout as well as UDP" OFF)

# --- Sources ---
add_executable(livox_bridge
  livox_bridge.cpp
)

# --- Include paths ---
# SDK2 installs headers into /usr/local/include by default
include_directories(/usr/local/include)

# --- Compiler definitions ---
if(BRIDGE_ENABLE_STDOUT)
  target_compile_definitions(livox_bridge PRIVATE BRIDGE_ENABLE_STDOUT=1)
endif()

# --- Find Livox SDK2 library ---
find_library(LIVOX_LIB
  NAMES livox_lidar_sdk_shared livox_lidar_sdk2 livox_lidar_sdk
  PATHS /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu
)
if(NOT LIVOX_LIB)
  message(FATAL_ERROR "Livox SDK2 library not found. Run 'sudo make install' in Livox-SDK2 and 'sudo ldconfig'.")
endif()

# --- Threads ---
find_package(Threads REQUIRED)

# --- Link ---
target_link_libraries(livox_bridge
  PRIVATE
  ${LIVOX_LIB}
  Threads::Threads
  dl
)

# --- RPATH so the binary finds the .so ---
set_target_properties(livox_bridge PROPERTIES
  BUILD_RPATH "/usr/local/lib"
  INSTALL_RPATH "/usr/local/lib"
)

# --- Install ---
install(TARGETS livox_bridge RUNTIME DESTINATION bin)

# --- Print summary ---
message(STATUS "Using Livox SDK2 lib: ${LIVOX_LIB}")
