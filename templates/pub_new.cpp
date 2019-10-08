#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for msg, info in msgs.items():
    else if(in->topicType == "`info.typespec.fullname`")
    {
        publisherProxy->publisher = node->create_publisher<`info.typespec.cpp_type()`>(in->topicName);
    }
#py endfor
