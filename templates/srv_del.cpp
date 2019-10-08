#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for srv, info in srvs.items():
    else if(serviceServerProxy->serviceType == "`info.typespec.fullname`")
    {
        auto srv = boost::any_cast< std::shared_ptr< rclcpp::Service<`info.typespec.cpp_type()`> > >(serviceServerProxy->service);
        srv = nullptr;
    }
#py endfor
