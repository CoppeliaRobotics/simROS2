#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for srv, info in srvs.items():
    else if(in->serviceType == "`info.typespec.fullname`")
    {
        auto cb = [=](const std::shared_ptr<rmw_request_id_t> request_header, const `info.typespec.cpp_type()`::Request::SharedPtr req, `info.typespec.cpp_type()`::Response::SharedPtr res) { ros_srv_callback__`info.typespec.normalized()`(request_header, req, res, serviceServerProxy); };
        serviceServerProxy->service = node->create_service<`info.typespec.cpp_type()`>(in->serviceName, cb);
    }
#py endfor
