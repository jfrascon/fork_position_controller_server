#include "joint_position_controller_server/joint_position_controller_client_single_goal_sequential.hpp"

#include <chrono>
#include <utility>

namespace joint_position_controller_server
{
  /**
   * @brief Build the sequential single-goal example client and load its parameters.
   *
   * This node demonstrates the recommended "well-behaved" client policy: keep only one client-side
   * goal active at a time, request cancelation before changing target, and send the next goal only
   * after the previous one has already reached a terminal state.
   */
  JointPositionControllerClientSingleGoalSequential::JointPositionControllerClientSingleGoalSequential(
    const rclcpp::NodeOptions& options):
    rclcpp::Node("joint_position_controller_client_single_goal_sequential", options)
  {
    this->declare_parameter<std::string>("action_name", "joint_position");
    this->declare_parameter<double>("first_position", 0.2);
    this->declare_parameter<double>("second_position", 0.05);
    this->declare_parameter<double>("second_goal_delay", 1.0);
    this->declare_parameter<double>("wait_for_server_timeout", 5.0);

    action_name_             = this->get_parameter("action_name").as_string();
    first_position_          = this->get_parameter("first_position").get_value<double>();
    second_position_         = this->get_parameter("second_position").get_value<double>();
    second_goal_delay_       = this->get_parameter("second_goal_delay").get_value<double>();
    wait_for_server_timeout_ = this->get_parameter("wait_for_server_timeout").get_value<double>();

    client_            = rclcpp_action::create_client<JointPosition>(this, action_name_);
    completion_future_ = completion_promise_.get_future().share();
  }

  /**
   * @brief Return the configured action name.
   * @return Action name.
   */
  const std::string& JointPositionControllerClientSingleGoalSequential::action_name() const
  {
    return action_name_;
  }

  /**
   * @brief Return the configured timeout while waiting for the action server.
   * @return Timeout in seconds.
   */
  double JointPositionControllerClientSingleGoalSequential::wait_for_server_timeout() const
  {
    return wait_for_server_timeout_;
  }

  /**
   * @brief Return the completion future consumed by the node executable.
   * @return Shared completion future.
   */
  std::shared_future<int> JointPositionControllerClientSingleGoalSequential::completion_future() const
  {
    return completion_future_;
  }

  /**
   * @brief Return the underlying ROS 2 action client.
   * @return Action client shared pointer.
   */
  rclcpp_action::Client<JointPositionControllerClientSingleGoalSequential::JointPosition>::SharedPtr
    JointPositionControllerClientSingleGoalSequential::client()
  {
    return client_;
  }

  /**
   * @brief Start the sequential demonstration flow.
   *
   * The first goal is sent immediately. A one-shot timer later asks for the second target. If the
   * first goal is still active by then, the client requests cancelation and waits for the cancel
   * result before sending the second goal.
   */
  void JointPositionControllerClientSingleGoalSequential::start()
  {
    send_goal(first_position_, "first_goal");

    const auto delay = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(second_goal_delay_));

    second_goal_timer_ = this->create_wall_timer(delay, [this]() {
      second_goal_timer_->cancel();
      request_goal_change(second_position_, "second_goal");
    });
  }

  /**
   * @brief Build the callback bundle shared by all sequential requests.
   * @return Action send-goal options.
   */
  rclcpp_action::Client<JointPositionControllerClientSingleGoalSequential::JointPosition>::SendGoalOptions
    JointPositionControllerClientSingleGoalSequential::create_send_goal_options()
  {
    typename rclcpp_action::Client<JointPosition>::SendGoalOptions options;
    options.goal_response_callback = std::bind(&JointPositionControllerClientSingleGoalSequential::goal_response_cb,
                                               this,
                                               std::placeholders::_1);
    options.feedback_callback      = std::bind(&JointPositionControllerClientSingleGoalSequential::feedback_cb,
                                               this,
                                               std::placeholders::_1,
                                               std::placeholders::_2);
    options.result_callback        = std::bind(&JointPositionControllerClientSingleGoalSequential::result_cb,
                                               this,
                                               std::placeholders::_1);
    return options;
  }

  /**
   * @brief Resolve the completion promise only once.
   * @param return_code Process return code to expose through completion_future().
   */
  void JointPositionControllerClientSingleGoalSequential::complete_once(int return_code)
  {
    if(completion_reported_)
    {
      return;
    }

    completion_reported_ = true;
    completion_promise_.set_value(return_code);
  }

  /**
   * @brief Ask for one new target while preserving the one-goal-at-a-time client policy.
   * @param position Requested joint position.
   * @param label Human-readable goal label used in logs.
   */
  void JointPositionControllerClientSingleGoalSequential::request_goal_change(double position, const std::string& label)
  {
    if(!current_goal_handle_)
    {
      send_goal(position, label);
      return;
    }

    pending_goal_position_ = position;
    pending_goal_label_    = label;

    if(cancel_in_progress_)
    {
      return;
    }

    cancel_in_progress_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Requesting cancelation of '%s' before sending '%s'.",
                current_goal_label_.c_str(),
                label.c_str());
    client_->async_cancel_goal(current_goal_handle_);
  }

  /**
   * @brief Send one new goal because no other client-side goal is currently active.
   * @param position Requested joint position.
   * @param label Human-readable goal label used in logs.
   */
  void JointPositionControllerClientSingleGoalSequential::send_goal(double position, const std::string& label)
  {
    JointPosition::Goal goal;
    goal.position = position;

    current_goal_label_ = label;

    RCLCPP_INFO(this->get_logger(), "Sending '%s' with target %.6f.", label.c_str(), position);
    client_->async_send_goal(goal, create_send_goal_options());
  }

  /**
   * @brief Report whether the currently requested goal was accepted by the server.
   * @param goal_handle Goal handle returned by the action client.
   */
  void JointPositionControllerClientSingleGoalSequential::goal_response_cb(
    const GoalHandleJointPosition::SharedPtr& goal_handle)
  {
    if(!goal_handle)
    {
      RCLCPP_WARN(this->get_logger(), "Goal '%s' was rejected by the server.", current_goal_label_.c_str());
      current_goal_handle_.reset();
      complete_once(1);
      return;
    }

    current_goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "Goal '%s' accepted by the server.", current_goal_label_.c_str());
  }

  /**
   * @brief Log feedback for the currently active sequential goal.
   * @param goal_handle Goal handle.
   * @param feedback Feedback message.
   */
  void JointPositionControllerClientSingleGoalSequential::feedback_cb(
    GoalHandleJointPosition::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const JointPosition::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(),
                "Feedback for '%s': target=%.6f current=%.6f error=%.6f",
                current_goal_label_.c_str(),
                feedback->target_position,
                feedback->current_position,
                feedback->position_error);
  }

  /**
   * @brief Handle the terminal result of the current goal and continue the sequence if needed.
   * @param result Wrapped action result.
   */
  void JointPositionControllerClientSingleGoalSequential::result_cb(const GoalHandleJointPosition::WrappedResult& result)
  {
    const std::string finished_goal_label = current_goal_label_;

    current_goal_handle_.reset();
    cancel_in_progress_ = false;

    switch(result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(),
                    "Goal '%s' succeeded: final_position=%.6f message='%s'",
                    finished_goal_label.c_str(),
                    result.result->final_position,
                    result.result->message.c_str());
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(),
                    "Goal '%s' aborted: final_position=%.6f message='%s'",
                    finished_goal_label.c_str(),
                    result.result->final_position,
                    result.result->message.c_str());
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(),
                    "Goal '%s' canceled: final_position=%.6f message='%s'",
                    finished_goal_label.c_str(),
                    result.result->final_position,
                    result.result->message.c_str());
        break;

      default:
        RCLCPP_ERROR(this->get_logger(),
                     "Goal '%s' finished with an unexpected result code.",
                     finished_goal_label.c_str());
        complete_once(1);
        return;
    }

    // This block implements the sequential policy explicitly. Only after the previous goal has
    // already reached one terminal state do we send the pending next goal.
    if(pending_goal_position_.has_value() && pending_goal_label_.has_value())
    {
      const double next_position = *pending_goal_position_;
      const auto next_label      = *pending_goal_label_;
      pending_goal_position_.reset();
      pending_goal_label_.reset();
      send_goal(next_position, next_label);
      return;
    }

    if(finished_goal_label == "second_goal")
    {
      complete_once(result.code == rclcpp_action::ResultCode::SUCCEEDED ? 0 : 1);
    }
  }
}  // namespace joint_position_controller_server
