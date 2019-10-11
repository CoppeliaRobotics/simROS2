cmake_minimum_required(VERSION 3.5)
project(sim_ros2_interface)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(test_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/CoppeliaRobotics-modules)
find_package(CoppeliaSim 3.6.1.2 REQUIRED)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/external)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/generated)
include_directories(${actionlib_msgs_INCLUDE_DIRS})
include_directories(${diagnostic_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${map_msgs_INCLUDE_DIRS})
include_directories(${nav_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${shape_msgs_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${test_msgs_INCLUDE_DIRS})
include_directories(${tf2_geometry_msgs_INCLUDE_DIRS})
include_directories(${tf2_msgs_INCLUDE_DIRS})
include_directories(${tf2_sensor_msgs_INCLUDE_DIRS})
include_directories(${trajectory_msgs_INCLUDE_DIRS})
include_directories(${visualization_msgs_INCLUDE_DIRS})
include_directories(${COPPELIASIM_INCLUDE_DIR})
include_directories(${COPPELIASIM_COMMON_DIR})

set(ROS_MSG_SRV_CACHE "${CMAKE_CURRENT_BINARY_DIR}/ros-msg-srv-defs.pickle")
add_custom_command(OUTPUT "${ROS_MSG_SRV_CACHE}" COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/tools/parse_messages_and_services.py" "${CMAKE_CURRENT_SOURCE_DIR}/meta/messages.txt" "${CMAKE_CURRENT_SOURCE_DIR}/meta/services.txt" "${ROS_MSG_SRV_CACHE}" DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/meta/messages.txt" "${CMAKE_CURRENT_SOURCE_DIR}/meta/services.txt" "${CMAKE_CURRENT_SOURCE_DIR}/tools/parse_messages_and_services.py")

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/templates/ ${CMAKE_CURRENT_SOURCE_DIR}/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" COMMAND python "${CMAKE_CURRENT_SOURCE_DIR}/external/simStubsGen/external/pycpp/pycpp.py" -p "cache_file=${ROS_MSG_SRV_CACHE}" -i "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}" -o "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}" -P "${CMAKE_CURRENT_SOURCE_DIR}/tools" DEPENDS "${ROS_MSG_SRV_CACHE}" "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}")
    set(generatedFiles ${generatedFiles} "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_ros_code DEPENDS ${generatedFiles})
coppeliasim_generate_stubs(${CMAKE_CURRENT_BINARY_DIR}/generated XML_FILE ${CMAKE_CURRENT_SOURCE_DIR}/meta/callbacks.xml LUA_FILE ${CMAKE_CURRENT_SOURCE_DIR}/simExtROS2Interface.lua)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/include/config.h)

set(SOURCES
    src/sim_ros2_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/external/simPlusPlus/Plugin.cpp
    ${CMAKE_CURRENT_BINARY_DIR}/generated/stubs.cpp
    src/ros_msg_builtin_io.cpp
    ${COPPELIASIM_EXPORTED_SOURCES}
    ${CMAKE_CURRENT_BINARY_DIR}/generated/callbacks.cpp
)
add_library(simExtROS2Interface SHARED ${SOURCES})
add_dependencies(simExtROS2Interface generate_ros_code)
target_link_libraries(simExtROS2Interface ${COPPELIASIM_LIBRARIES})
ament_target_dependencies(simExtROS2Interface rclcpp)
ament_target_dependencies(simExtROS2Interface actionlib_msgs)
ament_target_dependencies(simExtROS2Interface diagnostic_msgs)
ament_target_dependencies(simExtROS2Interface geometry_msgs)
ament_target_dependencies(simExtROS2Interface map_msgs)
ament_target_dependencies(simExtROS2Interface nav_msgs)
ament_target_dependencies(simExtROS2Interface sensor_msgs)
ament_target_dependencies(simExtROS2Interface shape_msgs)
ament_target_dependencies(simExtROS2Interface std_msgs)
ament_target_dependencies(simExtROS2Interface test_msgs)
ament_target_dependencies(simExtROS2Interface tf2_geometry_msgs)
ament_target_dependencies(simExtROS2Interface tf2_msgs)
ament_target_dependencies(simExtROS2Interface tf2_sensor_msgs)
ament_target_dependencies(simExtROS2Interface trajectory_msgs)
ament_target_dependencies(simExtROS2Interface visualization_msgs)

## This will produce the library directly in CoppeliaSim root folder. The default folder
## for the libsimExtROS2Interface.so file will not be devel/lib anymore.
#set_target_properties(simExtROS2Interface PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${COPPELIASIM_ROOT_DIR})

## install the library to COPPELIASIM_ROOT_DIR folder
## this command will install the generated library to the CoppeliaSim root folder no matter where the library is generated.
#install(TARGETS simExtROS2Interface DESTINATION ${COPPELIASIM_ROOT_DIR})

ament_package()
