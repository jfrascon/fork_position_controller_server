#pragma once

/**
 * @file fork_position_controller_client_single_goal_sequential.hpp
 * @brief Sequential single-goal example action client for the fork position controller server.
 */

#include <future>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "fork_position_controller_server/action/fork_position.hpp"

namespace fork_position_controller_server
{
  /**
   * @brief Example client that keeps at most one client-side goal alive at a time.
   *
   * The example first sends one goal. After a configurable delay, it requests a new target. If the
   * first goal is still active, the client cancels it, waits for the final cancel result, and only
   * then sends the next goal.
   */
  class ForkPositionControllerClientSingleGoalSequential: public rclcpp::Node
  {
    public:
    using ForkPosition           = fork_position_controller_server::action::ForkPosition;
    using GoalHandleForkPosition = rclcpp_action::ClientGoalHandle<ForkPosition>;

    /**
     * @brief Build the sequential single-goal example client node.
     * @param options ROS2 node options.
     */
    explicit ForkPositionControllerClientSingleGoalSequential(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

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
     * @brief Get the future that becomes ready when the example flow finishes.
     * @return Shared future containing the process return code.
     */
    std::shared_future<int> completion_future() const;

    /**
     * @brief Get the underlying ROS action client.
     * @return Action client shared pointer.
     */
    rclcpp_action::Client<ForkPosition>::SharedPtr client();

    /**
     * @brief Start the sequential example flow.
     */
    void start();

    private:
    /** @brief Action client used to communicate with the server. */
    rclcpp_action::Client<ForkPosition>::SharedPtr client_;

    /** @brief Timer that triggers the request for the second target. */
    rclcpp::TimerBase::SharedPtr second_goal_timer_;

    /** @brief Action name used by this example client. */
    std::string action_name_;

    /** @brief First target position sent by the example flow. */
    double first_position_{0.0};

    /** @brief Second target position sent after the first goal is canceled or completed. */
    double second_position_{0.0};

    /** @brief Delay in seconds before requesting the second target. */
    double second_goal_delay_{0.0};

    /** @brief Timeout while waiting for the action server to appear. */
    double wait_for_server_timeout_{0.0};

    /** @brief Handle of the goal currently owned by this client. */
    GoalHandleForkPosition::SharedPtr current_goal_handle_;

    /** @brief Human-readable label of the goal currently owned by this client. */
    std::string current_goal_label_;

    /** @brief Position of the goal that should be sent once the current goal finishes. */
    std::optional<double> pending_goal_position_;

    /** @brief Human-readable label of the pending goal. */
    std::optional<std::string> pending_goal_label_;

    /** @brief Prevents requesting cancelation multiple times for the same goal. */
    bool cancel_in_progress_{false};

    /** @brief Process return code published when the example finishes. */
    std::promise<int> completion_promise_;

    /** @brief Shared completion future consumed by the node executable. */
    std::shared_future<int> completion_future_;

    /** @brief Prevents resolving the completion promise more than once. */
    bool completion_reported_{false};

    /**
     * @brief Build the send-goal options shared by all requests in this example.
     * @return Action send-goal options.
     */
    rclcpp_action::Client<ForkPosition>::SendGoalOptions create_send_goal_options();

    /**
     * @brief Resolve the example completion promise once.
     * @param return_code Process return code exposed through completion_future().
     */
    void complete_once(int return_code);

    /**
     * @brief Request a new target while preserving the sequential single-goal policy.
     * @param position Requested fork position.
     * @param label Human-readable goal label used in logs.
     */
    void request_goal_change(double position, const std::string& label);

    /**
     * @brief Send one new goal request because no client-side goal is currently active.
     * @param position Requested fork position.
     * @param label Human-readable goal label used in logs.
     */
    void send_goal(double position, const std::string& label);

    /**
     * @brief Handle the action server response when one goal request is accepted or rejected.
     * @param goal_handle Goal handle returned by the action client.
     */
    void goal_response_cb(const GoalHandleForkPosition::SharedPtr& goal_handle);

    /**
     * @brief Log feedback from the goal currently active in this sequential client.
     * @param goal_handle Goal handle.
     * @param feedback Feedback message.
     */
    void feedback_cb(GoalHandleForkPosition::SharedPtr goal_handle,
                     const std::shared_ptr<const ForkPosition::Feedback> feedback);

    /**
     * @brief Handle the final result of the current goal and continue the sequence if needed.
     * @param result Wrapped action result.
     */
    void result_cb(const GoalHandleForkPosition::WrappedResult& result);
  };
}  // namespace fork_position_controller_server
