# minimum cmake version
cmake_minimum_required(VERSION 3.10)

# project name, version
project(RMT_agent VERSION 0.9.0)

# Include external project
execute_process(COMMAND git submodule update --init)
# Disable swig for mraa
option (BUILDSWIGNODE "Build swig node modules." OFF)
option (BUILDTESTS "Override the addition of tests" OFF)
add_subdirectory(external/mraa)

include_directories(${PROJECT_SOURCE_DIR}/external/mraa/api)
add_executable(agent_example agent_example.c)
target_link_libraries(agent_example PRIVATE rmt_agent mraa)