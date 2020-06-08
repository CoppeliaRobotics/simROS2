#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'msg':
    else if(subscriberProxy->topicType == "`interface.full_name`")
    {
        auto sub = boost::any_cast< std::shared_ptr< rclcpp::Subscription<`interface.cpp_type`> > >(subscriberProxy->subscription);
        sub = nullptr;
    }
#py endif
#py endfor
