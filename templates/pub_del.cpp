#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for msg, info in msgs.items():
    else if(publisherProxy->topicType == "`info.typespec.fullname`")
    {
        auto pub = boost::any_cast< std::shared_ptr< rclcpp::Publisher<`info.typespec.cpp_type()`> > >(publisherProxy->publisher);
        pub = nullptr;
    }
#py endfor
