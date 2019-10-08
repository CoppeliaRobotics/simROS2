#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for msg, info in msgs.items():
    else if(subscriberProxy->topicType == "`info.typespec.fullname`")
    {
        auto sub = boost::any_cast< std::shared_ptr< rclcpp::Subscription<`info.typespec.cpp_type()`> > >(subscriberProxy->subscription);
        sub = nullptr;
    }
#py endfor
