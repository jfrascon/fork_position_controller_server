#pragma once

/**
 * @file fork_position_controller_server.hpp
 * @brief Action server that commands a fork position through a backend topic.
 */

#include <chrono>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "fork_position_controller_interfaces/action/fork_position.hpp"

namespace fork_position_controller_server
{
  /**
   * @brief Action server that publishes a position command while a goal is active and monitors
   * convergence.
   *
   * ## Concurrency model
   *
   * All ROS callbacks (`joint_state_cb`, `handle_goal`, `handle_cancel`, `handle_accepted`) use
   * the node's default MutuallyExclusive callback group and are dispatched sequentially by a
   * `SingleThreadedExecutor`.
   *
   * Goal execution runs in a detached thread started from `handle_accepted()`. That thread calls
   * `execute()`, which owns the full goal lifecycle. It reads joint state under a mutex and
   * clears `has_active_goal_` when the goal terminates.
   *
   * ## `shared_from_this` and node lifetime
   *
   * `handle_accepted()` captures a `shared_ptr` to the node via `shared_from_this()` and passes
   * it into the detached execution thread. This is necessary because the thread outlives the
   * callback and must guarantee the node is not destroyed while `execute()` is still running.
   * `rclcpp::Node` inherits `std::enable_shared_from_this`, so `shared_from_this()` is always
   * valid here given that the node is owned by a `shared_ptr` (the standard rclcpp usage).
   */
  class ForkPositionControllerServer: public rclcpp::Node
  {
    public:
    using ForkPosition           = fork_position_controller_interfaces::action::ForkPosition;
    using GoalHandleForkPosition = rclcpp_action::ServerGoalHandle<ForkPosition>;

    /**
     * @brief Build the server node.
     * @param options ROS2 node options.
     * @throws std::invalid_argument If parameters are inconsistent.
     */
    explicit ForkPositionControllerServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    private:
    /**
     * @brief Build subscriptions, publishers, and action server.
     */
    void create_subs_pubs();

    /**
     * @brief Declare and validate all ROS parameters used by this node.
     */
    void declare_and_validate_parameters();

    /**
     * @brief Execute one goal until it succeeds, is canceled, or aborts.
     * @param goal_handle Goal handle of the specific goal that this execution thread owns.
     */
    void execute(const std::shared_ptr<GoalHandleForkPosition> goal_handle);

    /**
     * @brief Start execution of an accepted goal.
     * @param goal_handle Goal handle.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleForkPosition> goal_handle);

    /**
     * @brief Decide if a goal cancel request should be accepted.
     * @param goal_handle Goal handle.
     * @return ROS2 cancel response.
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleForkPosition> goal_handle);

    /**
     * @brief Decide if a goal should be accepted or rejected.
     * @param uuid Goal UUID.
     * @param goal Requested goal.
     * @return ROS2 goal response.
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const ForkPosition::Goal> goal);

    /**
     * @brief Process incoming JointState messages and update the measured fork position.
     * @param msg JointState message.
     */
    void joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

    /**
     * @brief Timer callback that publishes the current command position continuously.
     *
     * This callback runs at command_publish_frequency and reads pos_cmd_ (protected by mutex).
     * If pos_cmd_ is NaN, nothing is published. Otherwise, it publishes the commanded position
     * to the backend topic. This ensures continuous publication even after execute() terminates.
     */
    void pos_cmd_timer_cb();

    /**
     * @brief Publish one position command with the provided position.
     * @param position Desired backend position to publish.
     */
    void publish_pos_cmd(double position);

    /** @brief Action name exposed by this package. */
    static constexpr const char* action_name_{"fork_position"};

    /** @brief Publisher for backend position commands. */
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr command_pub_;

    /** @brief Timer that publishes position commands at command_publish_frequency. */
    rclcpp::TimerBase::SharedPtr pos_cmd_timer_;

    /** @brief Subscription to JointState messages. */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;

    /** @brief Action server that exposes the fork position command interface. */
    rclcpp_action::Server<ForkPosition>::SharedPtr action_server_;

    /** @brief Logger used by the action server callbacks. */
    rclcpp::Logger action_server_logger_;

    /** @brief Logger used by the JointState callback. */
    rclcpp::Logger joint_state_logger_;

    /** @brief Protects all shared runtime state. */
    mutable std::mutex mutex_;

    /** @brief Backend topic used to command the fork position. */
    std::string command_topic_;

    /** @brief Backend topic used to receive the measured joint state. */
    std::string joint_states_topic_;

    /** @brief Exact JointState entry to monitor. */
    std::string joint_name_;

    /** @brief Lower allowed goal position. */
    double lower_limit_{0.0};

    /** @brief Upper allowed goal position. */
    double upper_limit_{0.0};

    /** @brief Latest measured fork position received from JointState. */
    double current_pos_{0.0};

    /**
     * @brief Position command to be published continuously by the timer callback.
     *
     * Protected by mutex_. Written by execute() thread, read by pos_cmd_timer_cb().
     * NaN indicates no active goal or no command to publish. Valid values are published
     * continuously at command_publish_frequency to hold the fork position.
     */
    double pos_cmd_{std::numeric_limits<double>::quiet_NaN()};

    /** @brief Absolute convergence tolerance used for goal success. */
    double position_tolerance_{0.0};

    /** @brief Rate of command publication in Hertz. */
    double command_publish_frequency_{0.0};

    /** @brief Rate of action feedback publication in Hertz. */
    double feedback_publish_frequency_{0.0};

    /** @brief Rate of the execution loop (convergence checks, cancellation, timeouts) in Hertz. */
    double execution_loop_frequency_{0.0};

    /** @brief Maximum allowed goal execution time in seconds. */
    double goal_timeout_{0.0};

    /** @brief Maximum age of the latest JointState in seconds. */
    double joint_state_timeout_{0.0};

    /** @brief True once at least one valid JointState for the configured joint has been received. */
    bool current_pos_is_valid_{false};

    /** @brief Time stamp at which the latest valid JointState for the configured joint was received. */
    rclcpp::Time last_joint_state_time_{0, 0, RCL_ROS_TIME};

    /**
     * @brief True while the single execution slot for goals is reserved or in use.
     *
     * `handle_goal()` sets this flag to true as soon as a goal is accepted so no second goal can
     * slip through before ROS2 later calls `handle_accepted()`. `execute()` clears it when that
     * goal finishes, is canceled, or aborts.
     */
    bool has_active_goal_{false};
  };
}  // namespace fork_position_controller_server
