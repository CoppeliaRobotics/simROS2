#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for srv, info in srvs.items():
    else if(serviceClientProxy->serviceType == "`info.typespec.fullname`")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<`info.typespec.cpp_type()`> > >(serviceClientProxy->client);
        cli = nullptr;
    }
#py endfor
