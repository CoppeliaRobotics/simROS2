#ifndef SIM_ROS2_INTERFACE_H_INCLUDED
#define SIM_ROS2_INTERFACE_H_INCLUDED

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

#include <boost/lexical_cast.hpp>
#include <boost/any.hpp>

#include "config.h"

#include <image_transport/image_transport.hpp>

#define PLUGIN_NAME "Ros2Interface"
#define PLUGIN_VERSION 5

struct ScriptCallback
{
    int scriptId;
    std::string name;
};

struct Proxy
{
    bool destroyAfterSimulationStop;
};

#include <ros_msg_builtin_io.h>

struct SubscriptionProxy : Proxy
{
    int handle;
    std::string topicName;
    std::string topicType;
    ScriptCallback topicCallback;
    boost::any subscription;
    image_transport::Subscriber imageTransportSubscription;
    WriteOptions wr_opt;
};

struct PublisherProxy : Proxy
{
    int handle;
    std::string topicName;
    std::string topicType;
    boost::any publisher;
    image_transport::Publisher imageTransportPublisher;
    ReadOptions rd_opt;
};

struct ClientProxy : Proxy
{
    int handle;
    std::string serviceName;
    std::string serviceType;
    boost::any client;
    ReadOptions rd_opt;
    WriteOptions wr_opt;
};

struct ServiceProxy : Proxy
{
    int handle;
    std::string serviceName;
    std::string serviceType;
    ScriptCallback serviceCallback;
    boost::any service;
    ReadOptions rd_opt;
    WriteOptions wr_opt;
};

struct ActionClientProxy : Proxy
{
    int handle;
    std::string actionName;
    std::string actionType;
    boost::any action_client;
    ReadOptions rd_opt;
    WriteOptions wr_opt;
};

struct ActionServerProxy : Proxy
{
    int handle;
    std::string actionName;
    std::string actionType;
    ScriptCallback handleGoalCallback;
    ScriptCallback handleCancelCallback;
    ScriptCallback handleAcceptedCallback;
    std::unordered_map<rclcpp_action::GoalUUID, std::shared_ptr<rclcpp_action::ServerGoalHandleBase> > goalHandles;
    boost::any action_server;
    ReadOptions rd_opt;
    WriteOptions wr_opt;
};

#include <stubs.h>
#include <callbacks.h>

#endif // SIM_ROS2_INTERFACE_H_INCLUDED
