#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionServerProxy->actionType == "`interface.full_name`")
    {
        auto actsrv = boost::any_cast< std::shared_ptr< rclcpp_action::Server<`interface.cpp_type`> > >(actionServerProxy->action_server);
        auto uuid = goalUUIDfromString(in->goalUUID);
        auto gh = dynamic_cast< rclcpp_action::ServerGoalHandle<`interface.cpp_type`>* >(actionServerProxy->goalHandles[uuid]);
        auto feedback = std::make_shared<`interface.feedback.cpp_type`>();
        read__`interface.feedback.cpp_type_normalized`(in->_stackID, feedback.get(), &(actionServerProxy->rd_opt));
        gh->publish_feedback(feedback);
    }
#py endif
#py endfor
