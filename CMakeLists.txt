cmake_minimum_required(VERSION 3.5)
project(sim_ros2_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pendulum_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/external/libPlugin/cmake)
find_package(CoppeliaSim 3.6.1.2 REQUIRED)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/sim_ros2_interface)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include/sim_ros2_interface)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${COPPELIASIM_INCLUDE_DIR})
include_directories(${COPPELIASIM_COMMON_DIR})

set(ROS_MSG_SRV_CACHE "${CMAKE_CURRENT_BINARY_DIR}/ros-msg-srv-defs.pickle")
add_custom_command(OUTPUT "${ROS_MSG_SRV_CACHE}" COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/tools/parse_messages_and_services.py" "${CMAKE_CURRENT_SOURCE_DIR}/meta/messages.txt" "${CMAKE_CURRENT_SOURCE_DIR}/meta/services.txt" "${ROS_MSG_SRV_CACHE}" DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/meta/messages.txt" "${CMAKE_CURRENT_SOURCE_DIR}/meta/services.txt" "${CMAKE_CURRENT_SOURCE_DIR}/tools/parse_messages_and_services.py")

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/templates/ ${CMAKE_CURRENT_SOURCE_DIR}/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/external/libPlugin/simStubsGen/external/pycpp/pycpp.py" -p "cache_file=${ROS_MSG_SRV_CACHE}" -i "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}" -o "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" -P "${CMAKE_CURRENT_SOURCE_DIR}/tools" DEPENDS "${ROS_MSG_SRV_CACHE}" "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}")
    set(generatedFiles ${generatedFiles} "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_ros_code DEPENDS ${generatedFiles})
coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/meta/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simExtROS2Interface.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/sim_ros2_interface/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/sim_ros2_interface/config.h)

set(SOURCES
    src/sim_ros2_interface.cpp
    src/ros_msg_builtin_io.cpp
    ${COPPELIASIM_EXPORTED_SOURCES}
    ${CMAKE_CURRENT_BINARY_DIR}/generated/callbacks.cpp
)
add_library(simExtROS2Interface ${SOURCES})
add_dependencies(simExtROS2Interface generate_ros_code)
target_include_directories(simExtROS2Interface PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(simExtROS2Interface ${COPPELIASIM_LIBRARIES})
#ament_target_dependencies(simExtROS2Interface rclcpp)
ament_target_dependencies(
    simExtROS2Interface
    "actionlib_msgs"
    "action_msgs"
    "diagnostic_msgs"
    "gazebo_msgs"
    "geometry_msgs"
    "lifecycle_msgs"
    "map_msgs"
    "nav_msgs"
    "pendulum_msgs"
    "rosgraph_msgs"
    "sensor_msgs"
    "shape_msgs"
    "std_msgs"
    "stereo_msgs"
    "tf2_geometry_msgs"
    "tf2_msgs"
    "trajectory_msgs"
    "unique_identifier_msgs"
    "visualization_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(simExtROS2Interface PRIVATE "SIM_ROS2_INTERFACE_BUILDING_LIBRARY")

install(
    DIRECTORY include/
    DESTINATION include
)
install(
    TARGETS simExtROS2Interface
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
    include
)
ament_export_interfaces(
    export_${PROJECT_NAME}
)
ament_export_libraries(
    simExtROS2Interface
)

ament_package()
