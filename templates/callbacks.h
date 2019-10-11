#ifndef SIM_ROS2_PLUGIN__CALLBACKS__H
#define SIM_ROS2_PLUGIN__CALLBACKS__H

#include <ros_msg_builtin_io.h>
#include <sim_ros2_interface.h>

#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for msg, info in msgs.items():
#include <`info.typespec.cpp_include()`>
#py endfor
#py for srv, info in srvs.items():
#include <`info.typespec.cpp_include()`>
#py endfor

#py for msg, info in msgssrvs.items():
void write__`info.typespec.normalized()`(const `info.typespec.cpp_type()`& msg, int stack, const WriteOptions *opt = NULL);
void read__`info.typespec.normalized()`(int stack, `info.typespec.cpp_type()` *msg, const ReadOptions *opt = NULL);
#py endfor
#py for msg, info in msgs.items():
void ros_callback__`info.typespec.normalized()`(const `info.typespec.cpp_type()`::SharedPtr msg, SubscriberProxy *proxy);
#py endfor
#py for srv, info in srvs.items():
bool ros_srv_callback__`info.typespec.normalized()`(const std::shared_ptr<rmw_request_id_t> request_header, const `info.typespec.cpp_type()`::Request::SharedPtr req, `info.typespec.cpp_type()`::Response::SharedPtr res, ServiceServerProxy *proxy);
#py endfor

#endif // SIM_ROS2_PLUGIN__CALLBACKS__H
