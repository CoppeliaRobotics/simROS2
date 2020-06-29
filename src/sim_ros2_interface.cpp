#include <sim_ros2_interface.h>
#include <simPlusPlus/Plugin.h>

#include <cstdlib>
#include <functional>
using namespace std::placeholders;

#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
//#include <cv_bridge/cv_bridge.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

struct unsupported_type : public std::exception
{
    std::string message;
    unsupported_type(const std::string &w, const std::string &t)
    {
        std::stringstream ss;
        ss << "Unsupported " << w << " type '" << t << "'. You may want to add it to meta/interfaces.txt and recompile the ROS2Interface plugin.";
        message = ss.str();
    }
    ~unsupported_type() throw() {}
    const char * what() const throw() {return message.c_str();}
};

class Plugin : public sim::Plugin
{
public:
    void onStart()
    {
        if(!initialize())
            throw std::runtime_error("failed to initialize ROS2 node");

        if(!registerScriptStuff())
            throw std::runtime_error("failed to register script stuff");

        setExtVersion("ROS2 Interface Plugin");
        setBuildDate(BUILD_DATE);
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
        shutdownTransientProxies();
    }

    bool shouldProxyBeDestroyedAfterSimulationStop(int scriptID)
    {
        if(simGetSimulationState() == sim_simulation_stopped)
            return false;
        int property;
        int associatedObject;
        if(simGetScriptProperty(scriptID, &property, &associatedObject) == -1)
            return false;
        if(property & sim_scripttype_threaded)
            property -= sim_scripttype_threaded;
        if(property == sim_scripttype_addonscript || property == sim_scripttype_addonfunction || property == sim_scripttype_customizationscript)
            return false;
        return true;
    }

    static void ros_imtr_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg, SubscriptionProxy *subscriptionProxy, Plugin *plugin)
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

        if(!imageTransportCallback(subscriptionProxy->topicCallback.scriptId, subscriptionProxy->topicCallback.name.c_str(), &in_args, &out_args))
        {
            std::cerr << "ros_imtr_callback: error: failed to call callback" << std::endl;
            return;
        }
    }

    void createSubscription(createSubscription_in *in, createSubscription_out *out)
    {
        SubscriptionProxy *subscriptionProxy = new SubscriptionProxy();
        subscriptionProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(in->_scriptID);
        subscriptionProxy->handle = subscriptionProxyNextHandle++;
        subscriptionProxy->topicName = in->topicName;
        subscriptionProxy->topicType = in->topicType;
        subscriptionProxy->topicCallback.scriptId = in->_scriptID;
        subscriptionProxy->topicCallback.name = in->topicCallback;
        subscriptionProxies[subscriptionProxy->handle] = subscriptionProxy;

        if(0) {}
#include <sub_new.cpp>
        else
        {
            throw unsupported_type("message", subscriptionProxy->topicType);
        }

        out->subscriptionHandle = subscriptionProxy->handle;
    }

    void shutdownSubscription(shutdownSubscription_in *in, shutdownSubscription_out *out)
    {
        if(subscriptionProxies.find(in->subscriptionHandle) == subscriptionProxies.end())
        {
            throw exception("invalid subscription handle");
        }

        SubscriptionProxy *subscriptionProxy = subscriptionProxies[in->subscriptionHandle];

        if(0) {}
#include <sub_del.cpp>
        else
        {
            throw unsupported_type("message", subscriptionProxy->topicType);
        }

        subscriptionProxies.erase(subscriptionProxy->handle);
        delete subscriptionProxy;
    }

    void subscriptionTreatUInt8ArrayAsString(subscriptionTreatUInt8ArrayAsString_in *in, subscriptionTreatUInt8ArrayAsString_out *out)
    {
        if(subscriptionProxies.find(in->subscriptionHandle) == subscriptionProxies.end())
        {
            throw exception("invalid subscription handle");
        }

        SubscriptionProxy *subscriptionProxy = subscriptionProxies[in->subscriptionHandle];
        subscriptionProxy->wr_opt.uint8array_as_string = true;
    }

    void createPublisher(createPublisher_in *in, createPublisher_out *out)
    {
        PublisherProxy *publisherProxy = new PublisherProxy();
        publisherProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(in->_scriptID);
        publisherProxy->handle = publisherProxyNextHandle++;
        publisherProxy->topicName = in->topicName;
        publisherProxy->topicType = in->topicType;
        publisherProxies[publisherProxy->handle] = publisherProxy;

        if(0) {}
#include <pub_new.cpp>
        else
        {
            throw unsupported_type("message", publisherProxy->topicType);
        }

        out->publisherHandle = publisherProxy->handle;
    }

    void shutdownPublisher(shutdownPublisher_in *in, shutdownPublisher_out *out)
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
            throw unsupported_type("message", publisherProxy->topicType);
        }

        publisherProxies.erase(publisherProxy->handle);
        delete publisherProxy;
    }

    void publisherTreatUInt8ArrayAsString(publisherTreatUInt8ArrayAsString_in *in, publisherTreatUInt8ArrayAsString_out *out)
    {
        if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
        {
            throw exception("invalid publisher handle");
        }

        PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];
        publisherProxy->rd_opt.uint8array_as_string = true;
    }

    void publish(publish_in *in, publish_out *out)
    {
        if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
        {
            throw exception("invalid publisher handle");
        }

        PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];

        simMoveStackItemToTop(in->_stackID, 0);

        if(0) {}
#include <pub_publish.cpp>
        else
        {
            throw unsupported_type("message", publisherProxy->topicType);
        }
    }

    void createClient(createClient_in *in, createClient_out *out)
    {
        ClientProxy *clientProxy = new ClientProxy();
        clientProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(in->_scriptID);
        clientProxy->handle = clientProxyNextHandle++;
        clientProxy->serviceName = in->serviceName;
        clientProxy->serviceType = in->serviceType;
        clientProxies[clientProxy->handle] = clientProxy;

        if(0) {}
#include <cli_new.cpp>
        else
        {
            throw unsupported_type("service", clientProxy->serviceType);
        }

        out->clientHandle = clientProxy->handle;
    }

    void shutdownClient(shutdownClient_in *in, shutdownClient_out *out)
    {
        if(clientProxies.find(in->clientHandle) == clientProxies.end())
        {
            throw exception("invalid service client handle");
        }

        ClientProxy *clientProxy = clientProxies[in->clientHandle];

        if(0) {}
#include <cli_del.cpp>
        else
        {
            throw unsupported_type("service", clientProxy->serviceType);
        }

        clientProxies.erase(clientProxy->handle);
        delete clientProxy;
    }

    void clientTreatUInt8ArrayAsString(clientTreatUInt8ArrayAsString_in *in, clientTreatUInt8ArrayAsString_out *out)
    {
        if(clientProxies.find(in->clientHandle) == clientProxies.end())
        {
            throw exception("invalid service client handle");
        }

        ClientProxy *clientProxy = clientProxies[in->clientHandle];
        clientProxy->rd_opt.uint8array_as_string = true;
        clientProxy->wr_opt.uint8array_as_string = true;
    }

    void waitForService(waitForService_in *in, waitForService_out *out)
    {
        if(clientProxies.find(in->clientHandle) == clientProxies.end())
        {
            throw exception("invalid service client handle");
        }

        ClientProxy *clientProxy = clientProxies[in->clientHandle];

        if(0) {}
#include <cli_wait.cpp>
        else
        {
            throw unsupported_type("service", clientProxy->serviceType);
        }
    }

    void call(call_in *in, call_out *out)
    {
        if(clientProxies.find(in->clientHandle) == clientProxies.end())
        {
            throw exception("invalid service client handle");
        }

        ClientProxy *clientProxy = clientProxies[in->clientHandle];

        simMoveStackItemToTop(in->_stackID, 0);

        if(0) {}
#include <cli_call.cpp>
        else
        {
            throw unsupported_type("service", clientProxy->serviceType);
        }
    }

    void createService(createService_in *in, createService_out *out)
    {
        ServiceProxy *serviceProxy = new ServiceProxy();
        serviceProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(in->_scriptID);
        serviceProxy->handle = serviceProxyNextHandle++;
        serviceProxy->serviceName = in->serviceName;
        serviceProxy->serviceType = in->serviceType;
        serviceProxy->serviceCallback.scriptId = in->_scriptID;
        serviceProxy->serviceCallback.name = in->serviceCallback;
        serviceProxies[serviceProxy->handle] = serviceProxy;

        if(0) {}
#include <srv_new.cpp>
        else
        {
            throw unsupported_type("service", serviceProxy->serviceType);
        }

        out->serviceHandle = serviceProxy->handle;
    }

    void shutdownService(shutdownService_in *in, shutdownService_out *out)
    {
        if(serviceProxies.find(in->serviceHandle) == serviceProxies.end())
        {
            throw exception("invalid service handle");
        }

        ServiceProxy *serviceProxy = serviceProxies[in->serviceHandle];

        if(0) {}
#include <srv_del.cpp>
        else
        {
            throw unsupported_type("service", serviceProxy->serviceType);
        }

        serviceProxies.erase(serviceProxy->handle);
        delete serviceProxy;
    }

    void serviceTreatUInt8ArrayAsString(serviceTreatUInt8ArrayAsString_in *in, serviceTreatUInt8ArrayAsString_out *out)
    {
        if(serviceProxies.find(in->serviceHandle) == serviceProxies.end())
        {
            throw exception("invalid service handle");
        }

        ServiceProxy *serviceProxy = serviceProxies[in->serviceHandle];
        serviceProxy->rd_opt.uint8array_as_string = true;
        serviceProxy->wr_opt.uint8array_as_string = true;
    }

    void createActionClient(createActionClient_in *in, createActionClient_out *out)
    {
        ActionClientProxy *actionClientProxy = new ActionClientProxy();
        actionClientProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(in->_scriptID);
        actionClientProxy->handle = actionClientProxyNextHandle++;
        actionClientProxy->actionName = in->actionName;
        actionClientProxy->actionType = in->actionType;
        actionClientProxy->goalResponseCallback.scriptId = in->_scriptID;
        actionClientProxy->goalResponseCallback.name = in->goalResponseCallback;
        actionClientProxy->feedbackCallback.scriptId = in->_scriptID;
        actionClientProxy->feedbackCallback.name = in->feedbackCallback;
        actionClientProxy->resultCallback.scriptId = in->_scriptID;
        actionClientProxy->resultCallback.name = in->resultCallback;
        actionClientProxies[actionClientProxy->handle] = actionClientProxy;

        if(0) {}
#include <actcli_new.cpp>
        else
        {
            throw unsupported_type("action", actionClientProxy->actionType);
        }

        out->actionClientHandle = actionClientProxy->handle;
    }

    void shutdownActionClient(shutdownActionClient_in *in, shutdownActionClient_out *out)
    {
        if(actionClientProxies.find(in->actionClientHandle) == actionClientProxies.end())
        {
            throw exception("invalid action client handle");
        }

        ActionClientProxy *actionClientProxy = actionClientProxies[in->actionClientHandle];

        if(0) {}
#include <actcli_del.cpp>
        else
        {
            throw unsupported_type("action", actionClientProxy->actionType);
        }

        actionClientProxies.erase(actionClientProxy->handle);
        delete actionClientProxy;
    }

    void actionClientTreatUInt8ArrayAsString(actionClientTreatUInt8ArrayAsString_in *in, actionClientTreatUInt8ArrayAsString_out *out)
    {
        if(actionClientProxies.find(in->actionClientHandle) == actionClientProxies.end())
        {
            throw exception("invalid action client handle");
        }

        ActionClientProxy *actionClientProxy = actionClientProxies[in->actionClientHandle];
        actionClientProxy->rd_opt.uint8array_as_string = true;
        actionClientProxy->wr_opt.uint8array_as_string = true;
    }

    void sendGoal(sendGoal_in *in, sendGoal_out *out)
    {
        if(actionClientProxies.find(in->actionClientHandle) == actionClientProxies.end())
        {
            throw exception("invalid action client handle");
        }

        ActionClientProxy *actionClientProxy = actionClientProxies[in->actionClientHandle];

        simMoveStackItemToTop(in->_stackID, 0);

        if(0) {}
#include <actcli_sendGoal.cpp>
        else
        {
            throw unsupported_type("action", actionClientProxy->actionType);
        }
    }

    void cancelLastGoal(cancelLastGoal_in *in, cancelLastGoal_out *out)
    {
        if(actionClientProxies.find(in->actionClientHandle) == actionClientProxies.end())
        {
            throw exception("invalid action client handle");
        }

        ActionClientProxy *actionClientProxy = actionClientProxies[in->actionClientHandle];

        if(0) {}
#include <actcli_cancelLastGoal.cpp>
        else
        {
            throw unsupported_type("action", actionClientProxy->actionType);
        }
    }

    void createActionServer(createActionServer_in *in, createActionServer_out *out)
    {
        ActionServerProxy *actionServerProxy = new ActionServerProxy();
        actionServerProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(in->_scriptID);
        actionServerProxy->handle = actionServerProxyNextHandle++;
        actionServerProxy->actionName = in->actionName;
        actionServerProxy->actionType = in->actionType;
        actionServerProxy->handleGoalCallback.scriptId = in->_scriptID;
        actionServerProxy->handleGoalCallback.name = in->handleGoalCallback;
        actionServerProxy->handleCancelCallback.scriptId = in->_scriptID;
        actionServerProxy->handleCancelCallback.name = in->handleCancelCallback;
        actionServerProxy->handleAcceptedCallback.scriptId = in->_scriptID;
        actionServerProxy->handleAcceptedCallback.name = in->handleAcceptedCallback;
        actionServerProxies[actionServerProxy->handle] = actionServerProxy;

        if(0) {}
#include <actsrv_new.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        out->actionServerHandle = actionServerProxy->handle;
    }

    void shutdownActionServer(shutdownActionServer_in *in, shutdownActionServer_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_del.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        actionServerProxies.erase(actionServerProxy->handle);
        delete actionServerProxy;
    }

    void actionServerTreatUInt8ArrayAsString(actionServerTreatUInt8ArrayAsString_in *in, actionServerTreatUInt8ArrayAsString_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];
        actionServerProxy->rd_opt.uint8array_as_string = true;
        actionServerProxy->wr_opt.uint8array_as_string = true;
    }

    template<typename Action>
    rclcpp_action::ServerGoalHandle<Action> * getGoalHandle(ActionServerProxy *actionServerProxy, const std::string &goalUUIDstr)
    {
        return getGoalHandle<Action>(actionServerProxy, goalUUIDfromString(goalUUIDstr));
    }

    template<typename Action>
    rclcpp_action::ServerGoalHandle<Action> * getGoalHandle(ActionServerProxy *actionServerProxy, const rclcpp_action::GoalUUID &goalUUID)
    {
        auto actsrv = boost::any_cast< std::shared_ptr< rclcpp_action::Server< Action > > >(actionServerProxy->action_server);
        auto it = actionServerProxy->goalHandles.find(goalUUID);
        if(it == actionServerProxy->goalHandles.end())
            throw std::runtime_error((boost::format("goal handle '%s' does not exist") % goalUUIDtoString(goalUUID)).str());
        auto goalHandleBase = it->second.get();
        return dynamic_cast< rclcpp_action::ServerGoalHandle<Action>* >(goalHandleBase);
    }

    void cleanupTerminalGoalHandles(ActionServerProxy *actionServerProxy)
    {
        /* FIXME: crash when actionServerActionSucceed is called */ return;

        for(auto it = actionServerProxy->goalHandles.begin(); it != actionServerProxy->goalHandles.end(); )
        {
            if(it->second->is_active())
                ++it;
            else
                actionServerProxy->goalHandles.erase(it);
        }
    }

    void actionServerPublishFeedback(actionServerPublishFeedback_in *in, actionServerPublishFeedback_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_action_publish_feedback.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }
    }

    void actionServerActionAbort(actionServerActionAbort_in *in, actionServerActionAbort_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_action_abort.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        cleanupTerminalGoalHandles(actionServerProxy);
    }

    void actionServerActionSucceed(actionServerActionSucceed_in *in, actionServerActionSucceed_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_action_succeed.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        cleanupTerminalGoalHandles(actionServerProxy);
    }

    void actionServerActionCanceled(actionServerActionCanceled_in *in, actionServerActionCanceled_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_action_canceled.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        cleanupTerminalGoalHandles(actionServerProxy);
    }

    void actionServerActionExecute(actionServerActionExecute_in *in, actionServerActionExecute_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_action_execute.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }

        cleanupTerminalGoalHandles(actionServerProxy);
    }

    void actionServerActionIsCanceling(actionServerActionIsCanceling_in *in, actionServerActionIsCanceling_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_action_is_canceling.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }
    }

    void actionServerActionIsActive(actionServerActionIsActive_in *in, actionServerActionIsActive_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_action_is_active.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }
    }

    void actionServerActionIsExecuting(actionServerActionIsExecuting_in *in, actionServerActionIsExecuting_out *out)
    {
        if(actionServerProxies.find(in->actionServerHandle) == actionServerProxies.end())
        {
            throw exception("invalid action server handle");
        }

        ActionServerProxy *actionServerProxy = actionServerProxies[in->actionServerHandle];

        if(0) {}
#include <actsrv_action_is_executing.cpp>
        else
        {
            throw unsupported_type("action", actionServerProxy->actionType);
        }
    }

    void sendTransform(sendTransform_in *in, sendTransform_out *out)
    {
        geometry_msgs::msg::TransformStamped t;
        read__geometry_msgs__msg__TransformStamped(in->_stackID, &t);
        tfbr->sendTransform(t);
    }

    void sendTransforms(sendTransforms_in *in, sendTransforms_out *out)
    {
        std::vector<geometry_msgs::msg::TransformStamped> v;

        simMoveStackItemToTopE(in->_stackID, 0);
        int i = simGetStackTableInfoE(in->_stackID, 0);
        if(i < 0)
            throw exception("error reading input argument 1 (origin): expected array");
        int oldsz = simGetStackSizeE(in->_stackID);
        simUnfoldStackTableE(in->_stackID);
        int sz = (simGetStackSizeE(in->_stackID) - oldsz + 1) / 2;
        for(int i = 0; i < sz; i++)
        {
            simMoveStackItemToTopE(in->_stackID, oldsz - 1);
            int j;
            read__int(in->_stackID, &j);
            simMoveStackItemToTop(in->_stackID, oldsz - 1);
            geometry_msgs::msg::TransformStamped t;
            read__geometry_msgs__msg__TransformStamped(in->_stackID, &t);
            v.push_back(t);
        }

        tfbr->sendTransform(v);
    }

    void imageTransportCreateSubscription(imageTransportCreateSubscription_in *in, imageTransportCreateSubscription_out *out)
    {
        SubscriptionProxy *subscriptionProxy = new SubscriptionProxy();
        subscriptionProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(in->_scriptID);
        subscriptionProxy->handle = subscriptionProxyNextHandle++;
        subscriptionProxy->topicName = in->topicName;
        subscriptionProxy->topicType = "@image_transport";
        subscriptionProxy->topicCallback.scriptId = in->_scriptID;
        subscriptionProxy->topicCallback.name = in->topicCallback;
        subscriptionProxies[subscriptionProxy->handle] = subscriptionProxy;

        subscriptionProxy->imageTransportSubscription = imtr->subscribe(in->topicName, in->queueSize, std::bind(ros_imtr_callback, _1, subscriptionProxy, this));

        if(!subscriptionProxy->imageTransportSubscription)
        {
            throw exception("failed creation of ROS ImageTransport subscription");
        }

        out->subscriptionHandle = subscriptionProxy->handle;
    }

    void imageTransportShutdownSubscription(imageTransportShutdownSubscription_in *in, imageTransportShutdownSubscription_out *out)
    {
        if(subscriptionProxies.find(in->subscriptionHandle) == subscriptionProxies.end())
        {
            throw exception("invalid subscription handle");
        }

        SubscriptionProxy *subscriptionProxy = subscriptionProxies[in->subscriptionHandle];
        //subscriptionProxy->imageTransportSubscription.shutdown();
        subscriptionProxies.erase(subscriptionProxy->handle);
        delete subscriptionProxy;
    }

    void imageTransportCreatePublisher(imageTransportCreatePublisher_in *in, imageTransportCreatePublisher_out *out)
    {
        PublisherProxy *publisherProxy = new PublisherProxy();
        publisherProxy->destroyAfterSimulationStop = shouldProxyBeDestroyedAfterSimulationStop(in->_scriptID);
        publisherProxy->handle = publisherProxyNextHandle++;
        publisherProxy->topicName = in->topicName;
        publisherProxy->topicType = "@image_transport";
        publisherProxies[publisherProxy->handle] = publisherProxy;

        publisherProxy->imageTransportPublisher = imtr->advertise(in->topicName, in->queueSize);

        if(!publisherProxy->imageTransportPublisher)
        {
            throw exception("failed creation of ROS ImageTransport publisher");
        }

        out->publisherHandle = publisherProxy->handle;
    }

    void imageTransportShutdownPublisher(imageTransportShutdownPublisher_in *in, imageTransportShutdownPublisher_out *out)
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

    void imageTransportPublish(imageTransportPublish_in *in, imageTransportPublish_out *out)
    {
        if(publisherProxies.find(in->publisherHandle) == publisherProxies.end())
        {
            throw exception("invalid publisher handle");
        }

        PublisherProxy *publisherProxy = publisherProxies[in->publisherHandle];

        sensor_msgs::msg::Image image_msg;
        rclcpp::Clock ros_clock(RCL_ROS_TIME);
        image_msg.header.stamp = ros_clock.now();
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
    }

    void getTime(getTime_in *in, getTime_out *out)
    {
        rcl_clock_type_t t = RCL_ROS_TIME;
        switch(in->clock_type)
        {
        case sim_ros2_clock_ros:    t = RCL_ROS_TIME;    break;
        case sim_ros2_clock_system: t = RCL_SYSTEM_TIME; break;
        case sim_ros2_clock_steady: t = RCL_STEADY_TIME; break;
        }
        rclcpp::Clock ros_clock(t);
        builtin_interfaces::msg::Time ros_now = ros_clock.now();
        out->time.sec = ros_now.sec;
        out->time.nanosec = ros_now.nanosec;
    }

    void getParamString(getParamString_in *in, getParamString_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
        if(out->exists)
        {
            auto &param = params_client->get_parameters({in->name}).front();
            if(param.get_type() == rclcpp::PARAMETER_STRING)
                out->value = param.get_value<rclcpp::PARAMETER_STRING>();
        }
    }

    void getParamInt(getParamInt_in *in, getParamInt_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
        if(out->exists)
        {
            auto &param = params_client->get_parameters({in->name}).front();
            if(param.get_type() == rclcpp::PARAMETER_INTEGER)
                out->value = param.get_value<rclcpp::PARAMETER_INTEGER>();
        }
    }

    void getParamDouble(getParamDouble_in *in, getParamDouble_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
        if(out->exists)
        {
            auto &param = params_client->get_parameters({in->name}).front();
            if(param.get_type() == rclcpp::PARAMETER_DOUBLE)
                out->value = param.get_value<rclcpp::PARAMETER_DOUBLE>();
        }
    }

    void getParamBool(getParamBool_in *in, getParamBool_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
        if(out->exists)
        {
            auto &param = params_client->get_parameters({in->name}).front();
            if(param.get_type() == rclcpp::PARAMETER_BOOL)
                out->value = param.get_value<rclcpp::PARAMETER_BOOL>();
        }
    }

    void setParamString(setParamString_in *in, setParamString_out *out)
    {
        params_client->set_parameters({rclcpp::Parameter(in->name, in->value)});
    }

    void setParamInt(setParamInt_in *in, setParamInt_out *out)
    {
        params_client->set_parameters({rclcpp::Parameter(in->name, in->value)});
    }

    void setParamDouble(setParamDouble_in *in, setParamDouble_out *out)
    {
        params_client->set_parameters({rclcpp::Parameter(in->name, in->value)});
    }

    void setParamBool(setParamBool_in *in, setParamBool_out *out)
    {
        params_client->set_parameters({rclcpp::Parameter(in->name, in->value)});
    }

    void hasParam(hasParam_in *in, hasParam_out *out)
    {
        out->exists = params_client->has_parameter(in->name);
    }

    void deleteParam(deleteParam_in *in, deleteParam_out *out)
    {
        if(params_client->has_parameter(in->name))
            params_client->set_parameters({rclcpp::Parameter(in->name, rclcpp::ParameterValue())});
    }

    void searchParam(searchParam_in *in, searchParam_out *out)
    {
        out->found = params_client->has_parameter(in->name);
    }

    void createInterface(createInterface_in *in, createInterface_out *out)
    {
        if(0) {}
#include <if_new.cpp>
        else
        {
            throw unsupported_type("interface", in->type);
        }
    }

    void getInterfaceConstants(getInterfaceConstants_in *in, getInterfaceConstants_out *out)
    {
        if(0) {}
#include <if_const.cpp>
        else
        {
            throw unsupported_type("interface", in->type);
        }
    }

    void supportedInterfaces(supportedInterfaces_in *in, supportedInterfaces_out *out)
    {
#include <if_list.cpp>
    }

    bool initialize()
    {
        rclcpp::init(0, nullptr);

        int node_name_length = 0;
        char *node_name = nullptr;
#if SIM_PROGRAM_FULL_VERSION_NB >= 3060104 // 3.6.1.rev4
        node_name = simGetStringNamedParam("ROS2Interface.nodeName", &node_name_length);
#endif

        node = rclcpp::Node::make_shared(node_name && node_name_length ? node_name : "sim_ros2_interface");

        if(node_name) simReleaseBuffer(node_name);

        tfbr = new tf2_ros::TransformBroadcaster(node);
        imtr = new image_transport::ImageTransport(node);

        params_client = std::make_shared<rclcpp::SyncParametersClient>(node);
        while(!params_client->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                log(sim_verbosity_errors, "Interrupted while waiting for parameters service");
                return false;
            }
            log(sim_verbosity_debug, "Parameters service not available. Waiting for it...");
        }

        return true;
    }

    void shutdown()
    {
        rclcpp::shutdown();
        params_client = nullptr;
        node = nullptr;

        delete imtr;
        delete tfbr;
    }

    void shutdownTransientSubscriptions()
    {
        std::vector<int> handles;

        for(std::map<int, SubscriptionProxy *>::iterator it = subscriptionProxies.begin(); it != subscriptionProxies.end(); ++it)
        {
            if(it->second->destroyAfterSimulationStop)
            {
                handles.push_back(it->first);
            }
        }

        for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
        {
            SubscriptionProxy *subscriptionProxy = subscriptionProxies[*it];
            if(!subscriptionProxy->subscription.empty())
            {
                shutdownSubscription_in in1;
                in1.subscriptionHandle = *it;
                shutdownSubscription_out out1;
                shutdownSubscription(&in1, &out1);
            }
            if(subscriptionProxy->imageTransportSubscription)
            {
                imageTransportShutdownSubscription_in in1;
                in1.subscriptionHandle = *it;
                imageTransportShutdownSubscription_out out1;
                imageTransportShutdownSubscription(&in1, &out1);
            }
        }
    }

    void shutdownTransientPublishers()
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
            PublisherProxy *publisherProxy = publisherProxies[*it];
            if(!publisherProxy->publisher.empty())
            {
                shutdownPublisher_in in1;
                in1.publisherHandle = *it;
                shutdownPublisher_out out1;
                shutdownPublisher(&in1, &out1);
            }
            if(publisherProxy->imageTransportPublisher)
            {
                imageTransportShutdownPublisher_in in1;
                in1.publisherHandle = *it;
                imageTransportShutdownPublisher_out out1;
                imageTransportShutdownPublisher(&in1, &out1);
            }
        }
    }

    void shutdownTransientClients()
    {
        std::vector<int> handles;

        for(std::map<int, ClientProxy *>::iterator it = clientProxies.begin(); it != clientProxies.end(); ++it)
        {
            if(it->second->destroyAfterSimulationStop)
            {
                handles.push_back(it->first);
            }
        }

        for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
        {
            shutdownClient_in in1;
            in1.clientHandle = *it;
            shutdownClient_out out1;
            shutdownClient(&in1, &out1);
        }
    }

    void shutdownTransientServices()
    {
        std::vector<int> handles;

        for(std::map<int, ServiceProxy *>::iterator it = serviceProxies.begin(); it != serviceProxies.end(); ++it)
        {
            if(it->second->destroyAfterSimulationStop)
            {
                handles.push_back(it->first);
            }
        }

        for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
        {
            shutdownService_in in1;
            in1.serviceHandle = *it;
            shutdownService_out out1;
            shutdownService(&in1, &out1);
        }
    }

    void shutdownTransientActionClients()
    {
        std::vector<int> handles;

        for(std::map<int, ActionClientProxy *>::iterator it = actionClientProxies.begin(); it != actionClientProxies.end(); ++it)
        {
            if(it->second->destroyAfterSimulationStop)
            {
                handles.push_back(it->first);
            }
        }

        for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
        {
            shutdownActionClient_in in1;
            in1.actionClientHandle = *it;
            shutdownActionClient_out out1;
            shutdownActionClient(&in1, &out1);
        }
    }

    void shutdownTransientActionServers()
    {
        std::vector<int> handles;

        for(std::map<int, ActionServerProxy *>::iterator it = actionServerProxies.begin(); it != actionServerProxies.end(); ++it)
        {
            if(it->second->destroyAfterSimulationStop)
            {
                handles.push_back(it->first);
            }
        }

        for(std::vector<int>::iterator it = handles.begin(); it != handles.end(); ++it)
        {
            shutdownActionServer_in in1;
            in1.actionServerHandle = *it;
            shutdownActionServer_out out1;
            shutdownActionServer(&in1, &out1);
        }
    }

    void shutdownTransientProxies()
    {
        shutdownTransientSubscriptions();
        shutdownTransientPublishers();
        shutdownTransientClients();
        shutdownTransientServices();
        shutdownTransientActionClients();
        shutdownTransientActionServers();
    }

private:
    int previousStopSimulationRequestCounter = -1;

    rclcpp::Node::SharedPtr node = nullptr;
    rclcpp::SyncParametersClient::SharedPtr params_client = nullptr;

    tf2_ros::TransformBroadcaster *tfbr = nullptr;
    image_transport::ImageTransport *imtr = nullptr;

    int subscriptionProxyNextHandle = 3562;
    int publisherProxyNextHandle = 7980;
    int clientProxyNextHandle = 26856;
    int serviceProxyNextHandle = 53749;
    int actionClientProxyNextHandle = 71848;
    int actionServerProxyNextHandle = 93012;

    std::map<int, SubscriptionProxy *> subscriptionProxies;
    std::map<int, PublisherProxy *> publisherProxies;
    std::map<int, ClientProxy *> clientProxies;
    std::map<int, ServiceProxy *> serviceProxies;
    std::map<int, ActionClientProxy *> actionClientProxies;
    std::map<int, ActionServerProxy *> actionServerProxies;
};

SIM_PLUGIN(PLUGIN_NAME, PLUGIN_VERSION, Plugin)
#include "stubsPlusPlus.cpp"
