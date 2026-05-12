#pragma once

/**
 * @file joint_position_controller_client.hpp
 * @brief Single-goal example action client for the joint position controller server.
 */

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "joint_position_controller_interfaces/action/joint_position.hpp"

namespace joint_position_controller_server
{
  /**
   * @brief Example client that sends one joint position goal and logs feedback and result.
   *
   * This client illustrates the simplest usage pattern for the package: send one goal, keep
   * receiving feedback for that goal, and wait for its final result.
   */
  class JointPositionControllerClient: public rclcpp::Node
  {
    public:
    using JointPosition           = joint_position_controller_interfaces::action::JointPosition;
    using GoalHandleJointPosition = rclcpp_action::ClientGoalHandle<JointPosition>;

    /**
     * @brief Build the example client node.
     * @param options ROS2 node options.
     */
    explicit JointPositionControllerClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Get the action goal built from the current node parameters.
     * @return Goal message.
     */
    JointPosition::Goal goal_from_parameters() const;

    /**
     * @brief Get the configured action name.
     * @return Action name.
     */
    const std::string& action_name() const;

    /**
     * @brief Get the configured timeout while waiting for the action server.
     * @return Timeout in seconds.
     */
    double wait_for_server_timeout() const;

    /**
     * @brief Build the send-goal options used by the example client.
     * @return Action send-goal options.
     */
    typename rclcpp_action::Client<JointPosition>::SendGoalOptions create_send_goal_options();

    /**
     * @brief Get the underlying ROS action client.
     * @return Action client shared pointer.
     */
    rclcpp_action::Client<JointPosition>::SharedPtr client();

    private:
    /** @brief Action client used to communicate with the server. */
    rclcpp_action::Client<JointPosition>::SharedPtr client_;

    /** @brief Action name used by this example client. */
    std::string action_name_;

    /** @brief Target position sent by this example client. */
    double position_{0.0};

    /** @brief Timeout while waiting for the server to appear. */
    double wait_for_server_timeout_{0.0};

    /**
     * @brief Log the server response when the goal request is accepted or rejected.
     * @param goal_handle Goal handle returned by the action client.
     */
    void goal_response_cb(const GoalHandleJointPosition::SharedPtr& goal_handle);

    /**
     * @brief Log action feedback messages.
     * @param goal_handle Goal handle.
     * @param feedback Feedback message.
     */
    void feedback_cb(GoalHandleJointPosition::SharedPtr goal_handle,
                     const std::shared_ptr<const JointPosition::Feedback> feedback);

    /**
     * @brief Log the final result reported by the action server.
     * @param result Wrapped action result.
     */
    void result_cb(const GoalHandleJointPosition::WrappedResult& result);
  };
}  // namespace joint_position_controller_server
