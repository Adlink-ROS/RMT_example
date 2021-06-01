# minimum cmake version
cmake_minimum_required(VERSION 3.10)

# project name, version
project(RMT_agent VERSION 0.9.0)

# Check Neuron Library exist or not
set(nlib_path "/opt/adlink/neuron-sdk/neuron-library")
if(EXISTS ${nlib_path})
  message("Neuron Library is found")
  add_definitions(-DSUPPORT_NLIB)
  include_directories(${nlib_path}/include)
  link_directories(${nlib_path}/lib)
  set(NLIB mraa)
else()
  message("Neuron Library is not found")
  set(NLIB "")
endif()

# Support ROS or not
option(USE_ROS "Build agent with ROS" ON)
if(USE_ROS)
  message("Build agent with ROS")
  add_definitions(-DSUPPORT_ROS)
  include_directories("/opt/ros/foxy/include")
  link_directories("/opt/ros/foxy/lib")
  set(ROSLIB rclcpp rcutils)
else()
  message("Build agent without ROS")
  set(ROSLIB "")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBNM REQUIRED libnm)
pkg_check_modules(GLIB REQUIRED glib-2.0)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp>=0.5)

include_directories(${CMAKE_SOURCE_DIR}/agent)
include_directories(${YAMLCPP_INCLUDE_DIRS})
add_executable(agent_example agent_example.cpp datainfo.cpp fileinfo.cpp)

target_link_libraries(agent_example PRIVATE rmt_agent ${NLIB} ${ROSLIB} ${YAMLCPP_LIBRARIES} ${LIBNM_LIBRARIES} ${GLIB_LIBRARIES})
target_include_directories(agent_example PRIVATE ${LIBNM_INCLUDE_DIRS} ${GLIB_INCLUDE_DIRS})
target_compile_options(agent_example PRIVATE ${LIBNM_CFLAGS_OTHER} ${GLIB_CFLAGS_OTHER})
# copy multi_agents.py to 'build/agent/'
add_custom_target(agent_link_target ALL COMMAND ${CMAKE_COMMAND} -E 
                  copy "${PROJECT_SOURCE_DIR}/multi_agents.py" "${PROJECT_BINARY_DIR}/multi_agents.py")

# copy neuronbot2_tasks dir to 'build/agent/neuronbot2_tasks'
add_custom_target(nb2_task_link_target ALL COMMAND ${CMAKE_COMMAND} -E 
copy_directory "${PROJECT_SOURCE_DIR}/neuronbot2_tasks" "${PROJECT_BINARY_DIR}/neuronbot2_tasks")

install(
  FILES
    ${CMAKE_BINARY_DIR}/agent_example
  DESTINATION 
    usr/bin
)
install(
  FILES
    ${CMAKE_SOURCE_DIR}/RMTagent.service
  DESTINATION 
    $ENV{HOME}/.config/systemd/user/
  )

# Package
set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_VERSION ${CMAKE_PROJECT_VERSION})
set(CPACK_PACKAGE_NAME "rmt_agent")
set(CPACK_SET_DESTDIR true)
set(CPACK_INSTALL_PREFIX "/")
set(CPACK_OUTPUT_FILE_PREFIX ${CMAKE_SOURCE_DIR}/build)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "ChenYing Kuo")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "RMT Agent Example")
execute_process(COMMAND dpkg --print-architecture OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CPACK_DEBIAN_PACKAGE_ARCHITECTURE)
set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_NAME}_${CPACK_PACKAGE_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
    "${CMAKE_CURRENT_SOURCE_DIR}/postinst")
include(CPack)