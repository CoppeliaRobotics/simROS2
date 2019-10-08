#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for srv, info in srvs.items():
    else if(in->serviceType == "`info.typespec.fullname`")
    {
        serviceClientProxy->client = node->create_client<`info.typespec.cpp_type()`>(in->serviceName);
    }
#py endfor
