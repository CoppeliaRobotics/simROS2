#py from parse_interfaces import *
#py interfaces = parse_interfaces(pycpp.params['interfaces_file'])
#py for interface_name, interface in interfaces.items():
#py if interface.tag == 'action':
    else if(actionClientProxy->actionType == "`interface.full_name`")
    {
        auto cli = boost::any_cast< std::shared_ptr< rclcpp_action::Client<`interface.cpp_type`> > >(actionClientProxy->action_client);
        if(!cli->wait_for_action_server(std::chrono::seconds(5)))
            throw exception("action server not available after wait");
        `interface.goal.cpp_type` goal_msg;
        read__`interface.goal.cpp_type_normalized`(p->stackID, &goal_msg, &(actionClientProxy->rd_opt));
        auto send_goal_options = rclcpp_action::Client<`interface.cpp_type`>::SendGoalOptions();
        send_goal_options.goal_response_callback = [=] (std::shared_future< std::shared_ptr< rclcpp_action::ClientGoalHandle<`interface.cpp_type`> > > future) -> void
        {
            actionGoalResponseCallback_in in1;
            actionGoalResponseCallback_out out1;
            auto goal_handle = future.get();
            in1.goalID = goal_handle ? goalUUIDtoString(goal_handle->get_goal_id()) : "";
            in1.accepted = !!goal_handle;
            actionGoalResponseCallback(p->scriptID, in->goalResponseCallback.c_str(), &in1, &out1);
        };
        send_goal_options.feedback_callback = [=] (rclcpp_action::ClientGoalHandle<`interface.cpp_type`>::SharedPtr, const std::shared_ptr<const `interface.feedback.cpp_type`> feedback) -> void
        {
            ros_action_callback__`interface.feedback.cpp_type_normalized`(p->scriptID, in->feedbackCallback.c_str(), feedback.get(), actionClientProxy);
        };
        send_goal_options.result_callback = [=] (const rclcpp_action::ClientGoalHandle<`interface.cpp_type`>::WrappedResult &result) -> void
        {
            int lua_code;
            switch(result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                lua_code = sim_ros2_action_result_code_succeeded;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                lua_code = sim_ros2_action_result_code_aborted;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                lua_code = sim_ros2_action_result_code_succeeded;
                break;
            default:
                lua_code = sim_ros2_action_result_code_succeeded;
                break;
            }
            ros_action_callback__`interface.result.cpp_type_normalized`(p->scriptID, in->feedbackCallback.c_str(), lua_code, result.result, actionClientProxy);
        };
        auto goal_handle_future = cli->async_send_goal(goal_msg, send_goal_options);
        //if(rclcpp::spin_until_future_complete(node, goal_handle_future) ==
        //        rclcpp::executor::FutureReturnCode::SUCCESS)
        //{
        //    ...
        //}
    }
#py endif
#py endfor
