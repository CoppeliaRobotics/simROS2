#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for msg, info in msgs.items():
    else if(in->topicType == "`info.typespec.fullname`")
    {
        auto cb = [=](const `info.typespec.cpp_type()`::SharedPtr msg) { ros_callback__`info.typespec.normalized()`(msg, subscriberProxy); };
        rclcpp::QoS qos = 10;
        subscriberProxy->subscription = node->create_subscription<`info.typespec.cpp_type()`>(in->topicName, qos, cb);
    }
#py endfor
