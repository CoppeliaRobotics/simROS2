#include <sim_ros2_interface.h>
#include <simPlusPlus/Plugin.h>

#include <cstdlib>
#include <functional>
using namespace std::placeholders;

//#include <tf/transform_broadcaster.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>

rclcpp::Node::SharedPtr node = nullptr;

//tf::TransformBroadcaster *tfbr = NULL;
//image_transport::ImageTransport *imtr = NULL;

int subscriberProxyNextHandle = 3562;
int publisherProxyNextHandle = 7980;
int serviceClientProxyNextHandle = 26856;
int serviceServerProxyNextHandle = 53749;

std::map<int, SubscriberProxy *> subscriberProxies;
std::map<int, PublisherProxy *> publisherProxies;
std::map<int, ServiceClientProxy *> serviceClientProxies;
std::map<int, ServiceServerProxy *> serviceServerProxies;

bool shouldProxyBeDestroyedAfterSimulationStop(SScriptCallBack *p)
{
    if(simGetSimulationState() == sim_simulation_stopped)
        return false;
    int property;
    int associatedObject;
    if(simGetScriptProperty(p->scriptID, &property, &associatedObject) == -1)
        return false;
    if(property & sim_scripttype_threaded)
        property -= sim_scripttype_threaded;
    if(property == sim_scripttype_addonscript || property == sim_scripttype_addonfunction || property == sim_scripttype_customizationscript)
        return false;
    return true;
}

#if 0
void ros_imtr_callback(const sensor_msgs::ImageConstPtr& msg, SubscriberProxy *subscriberProxy)
{
    if(msg->is_bigendian)
    {
        std::cerr << "ros_imtr_callback: error: big endian image not supported" << std::endl;
        return;
    }

    int data_len = msg->step * msg->height;

    imageTransportCallback_in in_args;
    imageTransportCallback_out out_args;

    in_args.width = msg->width;
    in_args.height = msg->height;
    in_args.data.resize(data_len);

    for(unsigned int i = 0; i < msg->height; i++)
    {
        int msg_idx = (msg->height - i - 1) * msg->step;
        int buf_idx = i * msg->step;
        for(unsigned int j = 0; j < msg->step; j++)
        {
            in_args.data[buf_idx + j] = msg->data[msg_idx + j];
        }
    }

    if(!imageTransportCallback(subscriberProxy->topicCallback.scriptId, subscriberProxy->topicCallback.name.c_str(), &in_args, &out_args))
    {
        std::cerr << "ros_imtr_callback: error: failed to call callback" << std::endl;
        return;
    }
}
#endif

void subscribe(SScriptCallBack * p, const char * cmd, subscribe_in * in, subscribe_out * out)
{
    SubscriberProxy *subscriberProxy = new SubscriberProxy();
    subscriberProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(p);
    subscriberProxy->handle = subscriberProxyNextHandle++;
    subscriberProxy->topicName = in->topicName;
    subscriberProxy->topicType = in->topicType;
    subscriberProxy->topicCallback.scriptId = p->scriptID;
    subscriberProxy->topicCallback.name = in->topicCallback;
    subscriberProxies[subscriberProxy->handle] = subscriberProxy;

    if(0) {}
#include <sub_new.cpp>
    else
    {
        throw exception("unsupported message type. please edit and recompile ROS plugin");
    }

    out->subscriberHandle = subscriberProxy->handle;
}

void shutdownSubscriber(SScriptCallBack * p, const char * cmd, shutdownSubscriber_in * in, shutdownSubscriber_out * out)
{
    if(subscriberProxies.find(in->subscriberHandle) == subscriberProxies.end())
    {
        throw exception("invalid subscriber handle");
    }

    SubscriberProxy *subscriberProxy = subscriberProxies[in->subscriberHandle];

    if(0) {}
#include <sub_del.cpp>
    else
    {
        throw exception("unsupported message type. please edit and recompile ROS plugin");
    }

    subscriberProxies.erase(subscriberProxy->handle);
    delete subscriberProxy;
}

void subscriberTreatUInt8ArrayAsString(SScriptCallBack * p, const char * cmd, subscriberTreatUInt8ArrayAsString_in * in, subscriberTreatUInt8ArrayAsString_out * out)
{
    if(subscriberProxies.find(in->subscriberHandle) == subscriberProxies.end())
    {
        throw exception("invalid subscriber handle");
    }

    SubscriberProxy *subscriberProxy = subscriberProxies[in->subscriberHandle];
    subscriberProxy->wr_opt.uint8array_as_string = true;
}

void advertise(SScriptCallBack * p, const char * cmd, advertise_in * in, advertise_out * out)
{
    PublisherProxy *publisherProxy = new PublisherProxy();
    publisherProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(p);
    publisherProxy->handle = publisherProxyNextHandle++;
    publisherProxy->topicName = in->topicName;
    publisherProxy->topicType = in->topicType;
    publisherProxies[publisherProxy->handle] = publisherProxy;

    if(0) {}
#include <pub_new.cpp>
    else
    {
        throw exception("unsupported message type. please edit and recompile ROS plugin");
    }

    out->publisherHandle = publisherProxy->handle;
}

void shutdownPublisher(SScriptCallBack * p, const char * cmd, shutdownPublisher_in * in, shutdownPublisher_out * out)
{
    if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
    {
        throw exception("invalid publisher handle");
    }

    PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];

    if(0) {}
#include <pub_del.cpp>
    else
    {
        throw exception("unsupported message type. please edit and recompile ROS plugin");
    }

    publisherProxies.erase(publisherProxy->handle);
    delete publisherProxy;
}

void publisherTreatUInt8ArrayAsString(SScriptCallBack * p, const char * cmd, publisherTreatUInt8ArrayAsString_in * in, publisherTreatUInt8ArrayAsString_out * out)
{
    if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
    {
        throw exception("invalid publisher handle");
    }

    PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];
    publisherProxy->rd_opt.uint8array_as_string = true;
}

void publish(SScriptCallBack * p, const char * cmd, publish_in * in, publish_out * out)
{
    if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
    {
        throw exception("invalid publisher handle");
    }

    PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];

    simMoveStackItemToTop(p->stackID, 0);

    if(0) {}
#include <pub_publish.cpp>
    else
    {
        throw exception("unsupported message type. please edit and recompile ROS plugin");
    }
}

void serviceClient(SScriptCallBack * p, const char * cmd, serviceClient_in * in, serviceClient_out * out)
{
    ServiceClientProxy *serviceClientProxy = new ServiceClientProxy();
    serviceClientProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(p);
    serviceClientProxy->handle = serviceClientProxyNextHandle++;
    serviceClientProxy->serviceName = in->serviceName;
    serviceClientProxy->serviceType = in->serviceType;
    serviceClientProxies[serviceClientProxy->handle] = serviceClientProxy;

    if(0) {}
#include <cli_new.cpp>
    else
    {
        throw exception("unsupported service type. please edit and recompile ROS plugin");
    }

    out->serviceClientHandle = serviceClientProxy->handle;
}

void shutdownServiceClient(SScriptCallBack * p, const char * cmd, shutdownServiceClient_in * in, shutdownServiceClient_out * out)
{
    if(serviceClientProxies.find(in->serviceClientHandle) == serviceClientProxies.end())
    {
        throw exception("invalid service client handle");
    }

    ServiceClientProxy *serviceClientProxy = serviceClientProxies[in->serviceClientHandle];

    if(0) {}
#include <cli_del.cpp>
    else
    {
        throw exception("unsupported service type. please edit and recompile ROS plugin");
    }

    serviceClientProxies.erase(serviceClientProxy->handle);
    delete serviceClientProxy;
}

void serviceClientTreatUInt8ArrayAsString(SScriptCallBack * p, const char * cmd, serviceClientTreatUInt8ArrayAsString_in * in, serviceClientTreatUInt8ArrayAsString_out * out)
{
    if(serviceClientProxies.find(in->serviceClientHandle) == serviceClientProxies.end())
    {
        throw exception("invalid service client handle");
    }

    ServiceClientProxy *serviceClientProxy = serviceClientProxies[in->serviceClientHandle];
    serviceClientProxy->rd_opt.uint8array_as_string = true;
    serviceClientProxy->wr_opt.uint8array_as_string = true;
}

void call(SScriptCallBack * p, const char * cmd, call_in * in, call_out * out)
{
    if(serviceClientProxies.find(in->serviceClientHandle) == serviceClientProxies.end())
    {
        throw exception("invalid service client handle");
    }

    ServiceClientProxy *serviceClientProxy = serviceClientProxies[in->serviceClientHandle];

    simMoveStackItemToTop(p->stackID, 0);

    if(0) {}
#include <cli_call.cpp>
    else
    {
        throw exception("unsupported service type. please edit and recompile ROS plugin");
    }
}

void advertiseService(SScriptCallBack * p, const char * cmd, advertiseService_in * in, advertiseService_out * out)
{
    ServiceServerProxy *serviceServerProxy = new ServiceServerProxy();
    serviceServerProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(p);
    serviceServerProxy->handle = serviceServerProxyNextHandle++;
    serviceServerProxy->serviceName = in->serviceName;
    serviceServerProxy->serviceType = in->serviceType;
    serviceServerProxy->serviceCallback.scriptId = p->scriptID;
    serviceServerProxy->serviceCallback.name = in->serviceCallback;
    serviceServerProxies[serviceServerProxy->handle] = serviceServerProxy;

    if(0) {}
#include <srv_new.cpp>
    else
    {
        throw exception("unsupported service type. please edit and recompile ROS plugin");
    }

    out->serviceServerHandle = serviceServerProxy->handle;
}

void shutdownServiceServer(SScriptCallBack * p, const char * cmd, shutdownServiceServer_in * in, shutdownServiceServer_out * out)
{
    if(serviceServerProxies.find(in->serviceServerHandle) == serviceServerProxies.end())
    {
        throw exception("invalid service server handle");
    }

    ServiceServerProxy *serviceServerProxy = serviceServerProxies[in->serviceServerHandle];

    if(0) {}
#include <srv_del.cpp>
    else
    {
        throw exception("unsupported service type. please edit and recompile ROS plugin");
    }

    serviceServerProxies.erase(serviceServerProxy->handle);
    delete serviceServerProxy;
}

void serviceServerTreatUInt8ArrayAsString(SScriptCallBack * p, const char * cmd, serviceServerTreatUInt8ArrayAsString_in * in, serviceServerTreatUInt8ArrayAsString_out * out)
{
    if(serviceServerProxies.find(in->serviceServerHandle) == serviceServerProxies.end())
    {
        throw exception("invalid service server handle");
    }

    ServiceServerProxy *serviceServerProxy = serviceServerProxies[in->serviceServerHandle];
    serviceServerProxy->rd_opt.uint8array_as_string = true;
    serviceServerProxy->wr_opt.uint8array_as_string = true;
}

void sendTransform(SScriptCallBack * p, const char * cmd, sendTransform_in * in, sendTransform_out * out)
{
#if 0
    geometry_msgs::TransformStamped t;
    read__geometry_msgs__TransformStamped(p->stackID, &t);
    tfbr->sendTransform(t);
#endif
}

void sendTransforms(SScriptCallBack * p, const char * cmd, sendTransforms_in * in, sendTransforms_out * out)
{
#if 0
    std::vector<geometry_msgs::TransformStamped> v;

    simMoveStackItemToTopE(p->stackID, 0);
    int i = simGetStackTableInfoE(p->stackID, 0);
    if(i < 0)
        throw exception("error reading input argument 1 (origin): expected array");
    int oldsz = simGetStackSizeE(p->stackID);
    simUnfoldStackTableE(p->stackID);
    int sz = (simGetStackSizeE(p->stackID) - oldsz + 1) / 2;
    for(int i = 0; i < sz; i++)
    {
        simMoveStackItemToTopE(p->stackID, oldsz - 1);
        int j;
        read__int(p->stackID, &j);
        simMoveStackItemToTop(p->stackID, oldsz - 1);
        geometry_msgs::TransformStamped t;
        read__geometry_msgs__TransformStamped(p->stackID, &t);
        v.push_back(t);
    }
    
    tfbr->sendTransform(v);
#endif
}

void imageTransportSubscribe(SScriptCallBack *p, const char *cmd, imageTransportSubscribe_in *in, imageTransportSubscribe_out *out)
{
    SubscriberProxy *subscriberProxy = new SubscriberProxy();
    subscriberProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(p);
    subscriberProxy->handle = subscriberProxyNextHandle++;
    subscriberProxy->topicName = in->topicName;
    subscriberProxy->topicType = "@image_transport";
    subscriberProxy->topicCallback.scriptId = p->scriptID;
    subscriberProxy->topicCallback.name = in->topicCallback;
    subscriberProxies[subscriberProxy->handle] = subscriberProxy;

#if 0
    subscriberProxy->imageTransportSubscriber = imtr->subscribe(in->topicName, in->queueSize, boost::bind(ros_imtr_callback, _1, subscriberProxy));

    if(!subscriberProxy->imageTransportSubscriber)
    {
        throw exception("failed creation of ROS ImageTransport subscriber");
    }
#endif

    out->subscriberHandle = subscriberProxy->handle;
}

void imageTransportShutdownSubscriber(SScriptCallBack *p, const char *cmd, imageTransportShutdownSubscriber_in *in, imageTransportShutdownSubscriber_out *out)
{
    if(subscriberProxies.find(in->subscriberHandle) == subscriberProxies.end())
    {
        throw exception("invalid subscriber handle");
    }

    SubscriberProxy *subscriberProxy = subscriberProxies[in->subscriberHandle];
    //subscriberProxy->imageTransportSubscriber.shutdown();
    subscriberProxies.erase(subscriberProxy->handle);
    delete subscriberProxy;
}

void imageTransportAdvertise(SScriptCallBack *p, const char *cmd, imageTransportAdvertise_in *in, imageTransportAdvertise_out *out)
{
    PublisherProxy *publisherProxy = new PublisherProxy();
    publisherProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(p);
    publisherProxy->handle = publisherProxyNextHandle++;
    publisherProxy->topicName = in->topicName;
    publisherProxy->topicType = "@image_transport";
    publisherProxies[publisherProxy->handle] = publisherProxy;

#if 0
    publisherProxy->imageTransportPublisher = imtr->advertise(in->topicName, in->queueSize);

    if(!publisherProxy->imageTransportPublisher)
    {
        throw exception("failed creation of ROS ImageTransport publisher");
    }
#endif

    out->publisherHandle = publisherProxy->handle;
}

void imageTransportShutdownPublisher(SScriptCallBack *p, const char *cmd, imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out)
{
    if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
    {
        throw exception("invalid publisher handle");
    }

    PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];
    //publisherProxy->imageTransportPublisher.shutdown();
    publisherProxies.erase(publisherProxy->handle);
    delete publisherProxy;
}

void imageTransportPublish(SScriptCallBack *p, const char *cmd, imageTransportPublish_in *in, imageTransportPublish_out *out)
{
#if 0
    if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
    {
        throw exception("invalid publisher handle");
    }

    PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];

    sensor_msgs::Image image_msg;
    image_msg.header.stamp = rclcpp::Time::now();
    image_msg.header.frame_id = in->frame_id;
    image_msg.encoding = sensor_msgs::image_encodings::RGB8;
    image_msg.width = in->width;
    image_msg.height = in->height;
    image_msg.step = image_msg.width * 3;

    int data_len = image_msg.step * image_msg.height;
    image_msg.data.resize(data_len);
    image_msg.is_bigendian = 0;

    for(unsigned int i = 0; i < image_msg.height; i++)
    {
        int msg_idx = (image_msg.height - i - 1) * image_msg.step;
        int buf_idx = i * image_msg.step;
        for(unsigned int j = 0; j < image_msg.step; j++)
        {
            image_msg.data[msg_idx + j] = in->data[buf_idx + j];
        }
    }

    publisherProxy->imageTransportPublisher.publish(image_msg);
#endif
}

void getTime(SScriptCallBack *p, const char *cmd, getTime_in *in, getTime_out *out)
{
    if(in->flag == 0)
    {
        auto now = std::chrono::steady_clock::now();
        auto d = std::chrono::duration<double>(now.time_since_epoch());
        out->time = d.count();
    }
}

void getParamString(SScriptCallBack *p, const char *cmd, getParamString_in *in, getParamString_out *out)
{
#if 0
    out->value = in->defaultValue;
    out->exists = rclcpp::param::get(in->name, out->value);
#endif
}

void getParamInt(SScriptCallBack *p, const char *cmd, getParamInt_in *in, getParamInt_out *out)
{
#if 0
    out->value = in->defaultValue;
    out->exists = rclcpp::param::get(in->name, out->value);
#endif
}

void getParamDouble(SScriptCallBack *p, const char *cmd, getParamDouble_in *in, getParamDouble_out *out)
{
#if 0
    out->value = in->defaultValue;
    out->exists = rclcpp::param::get(in->name, out->value);
#endif
}

void getParamBool(SScriptCallBack *p, const char *cmd, getParamBool_in *in, getParamBool_out *out)
{
#if 0
    out->value = in->defaultValue;
    out->exists = rclcpp::param::get(in->name, out->value);
#endif
}

void setParamString(SScriptCallBack *p, const char *cmd, setParamString_in *in, setParamString_out *out)
{
#if 0
    rclcpp::param::set(in->name, in->value);
#endif
}

void setParamInt(SScriptCallBack *p, const char *cmd, setParamInt_in *in, setParamInt_out *out)
{
#if 0
    rclcpp::param::set(in->name, in->value);
#endif
}

void setParamDouble(SScriptCallBack *p, const char *cmd, setParamDouble_in *in, setParamDouble_out *out)
{
#if 0
    rclcpp::param::set(in->name, in->value);
#endif
}

void setParamBool(SScriptCallBack *p, const char *cmd, setParamBool_in *in, setParamBool_out *out)
{
#if 0
    rclcpp::param::set(in->name, in->value);
#endif
}

void hasParam(SScriptCallBack *p, const char *cmd, hasParam_in *in, hasParam_out *out)
{
#if 0
    out->exists = rclcpp::param::has(in->name);
#endif
}

void deleteParam(SScriptCallBack *p, const char *cmd, deleteParam_in *in, deleteParam_out *out)
{
#if 0
    rclcpp::param::del(in->name);
#endif
}

void searchParam(SScriptCallBack *p, const char *cmd, searchParam_in *in, searchParam_out *out)
{
#if 0
    out->found = rclcpp::param::search(in->name, out->name);
#endif
}

bool initialize()
{
    rclcpp::init(0, nullptr);

    int node_name_length = 0;
    char *node_name = nullptr;
#if SIM_PROGRAM_FULL_VERSION_NB >= 3060104 // 3.6.1.rev4
    node_name = simGetStringNamedParam("Ros2Interface.nodeName", &node_name_length);
#endif

    node = rclcpp::Node::make_shared(node_name && node_name_length ? node_name : "sim_ros2_interface");

    if(node_name) simReleaseBuffer(node_name);

    // in ROS2 we have no such thing, so this check is not performed:
    //if(!ros::master::check())
    //    return false;

    //tfbr = new tf::TransformBroadcaster();
    //imtr = new image_transport::ImageTransport(*nh);

    return true;
}

void shutdown()
{
    rclcpp::shutdown();
    node = nullptr;

    //delete imtr;
    //delete tfbr;
}

void shutdownTransientSubscribers(SScriptCallBack *p)
{
    std::vector<int> handles;

    for(std::map<int, SubscriberProxy *>::iterator it = subscriberProxies.begin(); it != subscriberProxies.end(); ++it)
    {
        if(it->second->destroyAfterSimulationStop)
        {
            handles.push_back(it->first);
        }
    }

    for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
    {
        SubscriberProxy *subscriberProxy = subscriberProxies[*it];
        //if(proxy->subscription)
            shutdownSubscriber(p, *it);
        //if(proxy->imageTransportSubscriber)
        //    imageTransportShutdownSubscriber(p, *it);
    }
}

void shutdownTransientPublishers(SScriptCallBack *p)
{
    std::vector<int> handles;

    for(std::map<int, PublisherProxy *>::iterator it = publisherProxies.begin(); it != publisherProxies.end(); ++it)
    {
        if(it->second->destroyAfterSimulationStop)
        {
            handles.push_back(it->first);
        }
    }

    for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
    {
        PublisherProxy *proxy = publisherProxies[*it];
        //if(proxy->publisher)
            shutdownPublisher(p, *it);
        //if(proxy->imageTransportPublisher)
        //    imageTransportShutdownPublisher(p, *it);
    }
}

void shutdownTransientServiceClients(SScriptCallBack *p)
{
    std::vector<int> handles;

    for(std::map<int, ServiceClientProxy *>::iterator it = serviceClientProxies.begin(); it != serviceClientProxies.end(); ++it)
    {
        if(it->second->destroyAfterSimulationStop)
        {
            handles.push_back(it->first);
        }
    }

    for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
    {
        shutdownServiceClient(p, *it);
    }
}

void shutdownTransientServiceServers(SScriptCallBack *p)
{
    std::vector<int> handles;

    for(std::map<int, ServiceServerProxy *>::iterator it = serviceServerProxies.begin(); it != serviceServerProxies.end(); ++it)
    {
        if(it->second->destroyAfterSimulationStop)
        {
            handles.push_back(it->first);
        }
    }

    for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
    {
        shutdownServiceServer(p, *it);
    }
}

void shutdownTransientProxies(SScriptCallBack *p)
{
    shutdownTransientSubscribers(p);
    shutdownTransientPublishers(p);
    shutdownTransientServiceClients(p);
    shutdownTransientServiceServers(p);
}

class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        if(!initialize())
            throw std::runtime_error("failed to initialize ROS2 node");

        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        simSetModuleInfo(PLUGIN_NAME, 0, "ROS2 Interface Plugin", 0);
        simSetModuleInfo(PLUGIN_NAME, 1, BUILD_DATE, 0);
    }

    void onEnd()
    {
        shutdown();
    }

    void onInstancePass(const sim::InstancePassFlags &flags, bool first)
    {
        rclcpp::spin_some(node);
    }

    void onMainScriptAboutToBeCalled(int &out)
    {

        int stopSimulationRequestCounter;
        simGetIntegerParameter(sim_intparam_stop_request_counter, &stopSimulationRequestCounter);
        simBool doNotRun = simGetBoolParameter(sim_boolparam_rosinterface_donotrunmainscript);
        if(doNotRun > 0)
        {
            if(previousStopSimulationRequestCounter == -1)
                previousStopSimulationRequestCounter = stopSimulationRequestCounter;
            if(previousStopSimulationRequestCounter == stopSimulationRequestCounter)
                out = 0; // this tells CoppeliaSim that we don't wanna execute the main script
        }
        else
            previousStopSimulationRequestCounter = -1;
    }

    void onSimulationAboutToStart()
    {
        previousStopSimulationRequestCounter = -1;
    }

    void onSimulationEnded()
    {
        shutdownTransientProxies(NULL /* XXX: which SScriptCallBack struct? */);
    }

private:
    int previousStopSimulationRequestCounter = -1;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
