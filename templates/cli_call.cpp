#py import parse_messages_and_services as p
#py msgs, srvs, msgssrvs = p.load_cache(pycpp.params['cache_file'])
#py for srv, info in srvs.items():
    else if(serviceClientProxy->serviceType == "`info.typespec.fullname`")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp::Client<`info.typespec.cpp_type()`> > >(serviceClientProxy->client);
        auto req = std::make_shared<`info.typespec.cpp_type()`::Request>();
        read__`info.typespec.normalized()`Request(p->stackID, req.get(), &(serviceClientProxy->rd_opt));
        auto result = cli->async_send_request(req);
        if(rclcpp::spin_until_future_complete(node, result) ==
                rclcpp::executor::FutureReturnCode::SUCCESS)
        {
            auto resp = result.get();
            write__`info.typespec.normalized()`Response(*resp, p->stackID, &(serviceClientProxy->wr_opt));
        }
        else
        {
            throw exception("failed to call service `info.typespec.fullname`");
        }
    }
#py endfor
