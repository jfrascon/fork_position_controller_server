#include "joint_position_controller_server/joint_position_controller_client.hpp"

#include <chrono>

namespace joint_position_controller_server
{
  /**
   * @brief Build the single-goal example client and load its parameters.
   *
   * The node keeps the client-side flow intentionally simple: it sends one goal, keeps logging the
   * feedback of that goal, and waits for its terminal result.
   */
  JointPositionControllerClient::JointPositionControllerClient(const rclcpp::NodeOptions& options):
    rclcpp::Node("joint_position_controller_client", options)
  {
    this->declare_parameter<std::string>("action_name", "joint_position");
    this->declare_parameter<double>("position", 0.0);
    this->declare_parameter<double>("wait_for_server_timeout", 5.0);

    action_name_             = this->get_parameter("action_name").as_string();
    position_                = this->get_parameter("position").get_value<double>();
    wait_for_server_timeout_ = this->get_parameter("wait_for_server_timeout").get_value<double>();

    client_ = rclcpp_action::create_client<JointPosition>(this, action_name_);
  }

  /**
   * @brief Convert the configured target position into one action goal message.
   * @return Goal message built from the current node parameters.
   */
  JointPositionControllerClient::JointPosition::Goal JointPositionControllerClient::goal_from_parameters() const
  {
    JointPosition::Goal goal;
    goal.position = position_;
    return goal;
  }

  /**
   * @brief Return the action name that this example client uses.
   * @return Action name.
   */
  const std::string& JointPositionControllerClient::action_name() const
  {
    return action_name_;
  }

  /**
   * @brief Return the server wait timeout configured for this example client.
   * @return Timeout in seconds.
   */
  double JointPositionControllerClient::wait_for_server_timeout() const
  {
    return wait_for_server_timeout_;
  }

  /**
   * @brief Build the callback bundle used for the single-goal action request.
   * @return Action send-goal options.
   */
  typename rclcpp_action::Client<JointPositionControllerClient::JointPosition>::SendGoalOptions
    JointPositionControllerClient::create_send_goal_options()
  {
    typename rclcpp_action::Client<JointPosition>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&JointPositionControllerClient::goal_response_cb,
                                               this,
                                               std::placeholders::_1);
    options.feedback_callback      = std::bind(&JointPositionControllerClient::feedback_cb,
                                               this,
                                               std::placeholders::_1,
                                               std::placeholders::_2);
    options.result_callback        = std::bind(&JointPositionControllerClient::result_cb, this, std::placeholders::_1);
    return options;
  }

  /**
   * @brief Return the underlying ROS 2 action client.
   * @return Action client shared pointer.
   */
  rclcpp_action::Client<JointPositionControllerClient::JointPosition>::SharedPtr JointPositionControllerClient::client()
  {
    return client_;
  }

  /**
   * @brief Report whether the single goal was accepted by the server.
   * @param goal_handle Goal handle returned by the action client.
   */
  void JointPositionControllerClient::goal_response_cb(const GoalHandleJointPosition::SharedPtr& goal_handle)
  {
    if(!goal_handle)
    {
      RCLCPP_WARN(this->get_logger(), "Goal was rejected by the server.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal accepted by the server.");
  }

  /**
   * @brief Log the feedback that belongs to the only goal managed by this example client.
   * @param goal_handle Goal handle.
   * @param feedback Feedback message.
   */
  void JointPositionControllerClient::feedback_cb(GoalHandleJointPosition::SharedPtr /*goal_handle*/,
                                                 const std::shared_ptr<const JointPosition::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(),
                "Feedback: target=%.6f current=%.6f error=%.6f",
                feedback->target_position,
                feedback->current_position,
                feedback->position_error);
  }

  /**
   * @brief Log the terminal result of the only goal managed by this example client.
   * @param result Wrapped action result.
   */
  void JointPositionControllerClient::result_cb(const GoalHandleJointPosition::WrappedResult& result)
  {
    // The single-goal example intentionally maps each terminal result code to one log branch so
    // users can see the difference between success, abort, and cancelation.
    switch(result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(),
                    "Goal succeeded: success=%s final_position=%.6f message='%s'",
                    result.result->success ? "true" : "false",
                    result.result->final_position,
                    result.result->message.c_str());
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(),
                    "Goal aborted: success=%s final_position=%.6f message='%s'",
                    result.result->success ? "true" : "false",
                    result.result->final_position,
                    result.result->message.c_str());
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(),
                    "Goal canceled: success=%s final_position=%.6f message='%s'",
                    result.result->success ? "true" : "false",
                    result.result->final_position,
                    result.result->message.c_str());
        break;

      default:
        RCLCPP_ERROR(this->get_logger(), "Unexpected action result code.");
        break;
    }
  }
}  // namespace joint_position_controller_server
