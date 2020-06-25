#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionServerProxy->actionType == "`interface.full_name`")
    {
        auto actsrv = boost::any_cast< std::shared_ptr< rclcpp_action::Server<`interface.cpp_type`> > >(actionServerProxy->action_server);
        auto uuid = goalUUIDfromString(in->goalUUID);
        auto gh = dynamic_cast< rclcpp_action::ServerGoalHandle<`interface.cpp_type`>* >(actionServerProxy->goalHandles[uuid]);
        out->result = gh->is_executing();
    }
#py endif
#py endfor
