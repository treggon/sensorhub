
cmake_minimum_required(VERSION 3.10)
project(livox_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(livox_bridge
  livox_bridge.cpp
)

# Headers
include_directories(/usr/local/include)

# Find the Livox SDK2 shared lib name actually installed on your Orin
find_library(LIVOX_LIB
  NAMES livox_lidar_sdk_shared livox_lidar_sdk2 livox_lidar_sdk
  PATHS /usr/local/lib /usr/lib /usr/lib/aarch64-linux-gnu
)

if(NOT LIVOX_LIB)
  message(FATAL_ERROR "Livox SDK2 library not found. Run 'sudo make install' in Livox-SDK2 and 'sudo ldconfig'.")
endif()

# Link pthreads via CMake’s Threads package
find_package(Threads REQUIRED)

# Link everything
target_link_libraries(livox_bridge
  PRIVATE
    ${LIVOX_LIB}
    Threads::Threads
    dl               # commonly required by SDKs using dlsym
    # rt             # only if you see unresolved realtime clock symbols
)

# Let the binary find the .so in /usr/local/lib at runtime
set_target_properties(livox_bridge PROPERTIES
  BUILD_RPATH "/usr/local/lib"
  INSTALL_RPATH "/usr/local/lib"
)

