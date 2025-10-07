cmake_minimum_required(VERSION 3.10)
project(psn_to_mavlink CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT DEFINED MAVLINK_INCLUDE_DIR)
  message(FATAL_ERROR "Set MAVLINK_INCLUDE_DIR to mavlink/c_library_v2")
endif()

# Normalize to an absolute path (avoids relative/.. quirks)
get_filename_component(MAVLINK_ABS "${MAVLINK_INCLUDE_DIR}" REALPATH)
message(STATUS "MAVLINK_INCLUDE_DIR = ${MAVLINK_ABS}")

# Sanity check (fail early if wrong)
if (NOT EXISTS "${MAVLINK_ABS}/common/mavlink.h")
  message(FATAL_ERROR "Did not find ${MAVLINK_ABS}/common/mavlink.h")
endif()

add_executable(psn_to_mavlink psn_to_mavlink.cpp)
target_include_directories(psn_to_mavlink PRIVATE "${MAVLINK_ABS}")

if (WIN32)
  target_compile_definitions(psn_to_mavlink PRIVATE WIN32_LEAN_AND_MEAN _WINSOCK_DEPRECATED_NO_WARNINGS)
  target_link_libraries(psn_to_mavlink PRIVATE ws2_32)
endif()
