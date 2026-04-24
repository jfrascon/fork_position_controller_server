#include "fork_position_controller_server/fork_position_controller_server.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace fork_position_controller_server
{
  ForkPositionControllerServer::ForkPositionControllerServer(const rclcpp::NodeOptions& options):
    rclcpp::Node("fork_position_controller_server", options),
    action_server_logger_(this->get_logger().get_child("action_server")),
    joint_state_logger_(this->get_logger().get_child("joint_state_cb"))
  {
    declare_and_validate_parameters();
    create_subs_pubs();

    RCLCPP_INFO(this->get_logger(), "ForkPositionControllerServer node initialized.");
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void ForkPositionControllerServer::create_subs_pubs()
  {
    command_pub_ = this->create_publisher<std_msgs::msg::Float64>(command_topic_, 10);

    // All callbacks (joint_state_cb, handle_goal, handle_cancel, handle_accepted) use the
    // node's default MutuallyExclusive callback group, so no explicit group is needed.
    // Sharing the same MutuallyExclusive group guarantees they never run concurrently.
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ForkPositionControllerServer::joint_state_cb, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<ForkPosition>(
      this,
      action_name_,
      std::bind(&ForkPositionControllerServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ForkPositionControllerServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ForkPositionControllerServer::handle_accepted, this, std::placeholders::_1));

    // Create a timer that publishes position commands at the configured frequency.
    // This ensures continuous publication even after execute() terminates, maintaining
    // the fork in its target/holding position.
    const auto timer_period = std::chrono::duration<double>(1.0 / command_publication_frequency_);
    pos_cmd_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
                                             std::bind(&ForkPositionControllerServer::pos_cmd_timer_cb, this));
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void ForkPositionControllerServer::declare_and_validate_parameters()
  {
    this->declare_parameter<double>("lower_limit");
    this->declare_parameter<double>("upper_limit");
    this->declare_parameter<double>("position_tolerance", 0.005);
    this->declare_parameter<double>("command_publication_frequency", 20.0);
    this->declare_parameter<double>("feedback_publication_frequency", 10.0);
    this->declare_parameter<double>("execution_loop_frequency", 30.0);
    this->declare_parameter<double>("goal_timeout", 10.0);
    this->declare_parameter<double>("joint_state_timeout", 1.0);
    this->declare_parameter<std::string>("joint_name");
    this->declare_parameter<std::string>("command_topic", "joint_commands/fork");
    this->declare_parameter<std::string>("joint_states_topic", "joint_states");

    command_topic_                  = this->get_parameter("command_topic").as_string();
    joint_states_topic_             = this->get_parameter("joint_states_topic").as_string();
    joint_name_                     = this->get_parameter("joint_name").as_string();
    lower_limit_                    = this->get_parameter("lower_limit").get_value<double>();
    upper_limit_                    = this->get_parameter("upper_limit").get_value<double>();
    position_tolerance_             = this->get_parameter("position_tolerance").get_value<double>();
    command_publication_frequency_  = this->get_parameter("command_publication_frequency").get_value<double>();
    feedback_publication_frequency_ = this->get_parameter("feedback_publication_frequency").get_value<double>();
    execution_loop_frequency_       = this->get_parameter("execution_loop_frequency").get_value<double>();
    goal_timeout_                   = this->get_parameter("goal_timeout").get_value<double>();
    joint_state_timeout_            = this->get_parameter("joint_state_timeout").get_value<double>();

    if(joint_name_.empty())
    {
      throw std::invalid_argument("Parameter 'joint_name' must not be empty.");
    }

    if(command_topic_.empty())
    {
      throw std::invalid_argument("Parameter 'command_topic' must not be empty.");
    }

    if(joint_states_topic_.empty())
    {
      throw std::invalid_argument("Parameter 'joint_states_topic' must not be empty.");
    }

    if(lower_limit_ > upper_limit_)
    {
      throw std::invalid_argument("Parameter 'lower_limit' must be less than or equal to 'upper_limit'.");
    }

    if(position_tolerance_ < 0.0)
    {
      throw std::invalid_argument("Parameter 'position_tolerance' must be non-negative.");
    }

    if(command_publication_frequency_ <= 0.0)
    {
      throw std::invalid_argument("Parameter 'command_publication_frequency' must be positive.");
    }

    if(feedback_publication_frequency_ <= 0.0)
    {
      throw std::invalid_argument("Parameter 'feedback_publication_frequency' must be positive.");
    }

    if(execution_loop_frequency_ <= 0.0)
    {
      throw std::invalid_argument("Parameter 'execution_loop_frequency' must be positive.");
    }

    if(execution_loop_frequency_ < std::max(command_publication_frequency_, feedback_publication_frequency_))
    {
      throw std::invalid_argument("Parameter 'execution_loop_frequency' must be greater than or equal to both "
                                  "'command_publication_frequency' and 'feedback_publication_frequency' to ensure "
                                  "publication deadlines are met.");
    }

    if(goal_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Parameter 'goal_timeout' must be positive.");
    }

    if(joint_state_timeout_ <= 0.0)
    {
      throw std::invalid_argument("Parameter 'joint_state_timeout' must be positive.");
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void ForkPositionControllerServer::pos_cmd_timer_cb()
  {
    // Read the commanded position under the mutex and publish it.
    // publish_pos_cmd() handles NaN internally (won't publish if NaN).
    double cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = pos_cmd_;
    }

    publish_pos_cmd(cmd);
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void ForkPositionControllerServer::execute(const std::shared_ptr<GoalHandleForkPosition> goal_handle)
  {
    const auto target_pos{goal_handle->get_goal()->position};

    // Capture the goal start time locally.
    const auto goal_start_time{this->now()};

    // Compute the period for feedback publication based on its frequency.
    const auto feedback_period{std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / feedback_publication_frequency_))};

    // The execution loop runs at its own frequency, which is independent of (but must be >= max of)
    // the command and feedback publication rates. This allows for responsive convergence checking,
    // cancellation detection, and timeout monitoring without being coupled to publication rates.
    const auto tick{std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / execution_loop_frequency_))};

    // Initialize the last-publish timestamp for feedback to the clock epoch (t=0). On the first
    // iteration, (now - last_feedback_time) will be enormous, so the feedback publication condition
    // triggers immediately and the goal does not sit silent for one full period before sending
    // its first feedback.
    rclcpp::Time last_feedback_time{0, 0, this->get_clock()->get_clock_type()};

    const auto result{std::make_shared<ForkPosition::Result>()};

    while(rclcpp::ok())
    {
      const auto now{this->now()};

      // Local variables to hold the current position state.
      // They are updated under the mutex from the shared state and then used outside the mutex for
      // the rest of the loop iteration to minimize the time spent holding the lock.
      auto current_pos{std::numeric_limits<double>::quiet_NaN()};
      auto current_pos_is_valid{false};
      auto last_joint_state_time{rclcpp::Time{0, 0, this->get_clock()->get_clock_type()}};

      // Store the state to the local variables under the mutex so the rest of the iteration works
      // on a consistent, stable copy without holding the lock.
      {
        std::lock_guard<std::mutex> lock(mutex_);
        current_pos           = current_pos_;
        current_pos_is_valid  = current_pos_is_valid_;
        last_joint_state_time = last_joint_state_time_;
      }

      // ERROR CONDITIONS: Check critical error conditions first before processing cancellation or
      // success. These represent system-level problems that must be detected immediately.

      // If the joint state is invalid, we cannot observe the system state or measure convergence,
      // so abort the goal with an appropriate message. This also prevents the controller from
      // publishing commands based on an invalid position that could be dangerous.
      if(!current_pos_is_valid)
      {
        result->success        = false;
        result->final_position = std::numeric_limits<double>::quiet_NaN();
        result->message        = "Goal aborted because the joint state is invalid.";

        goal_handle->abort(result);

        // Stop publishing commands and mark the goal as no longer active.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_         = std::numeric_limits<double>::quiet_NaN();
          has_active_goal_ = false;
        }

        return;
      }

      // If the joint state is too old, abort for the same reasons as above but with a different
      // message to indicate the specific problem.
      if((now - last_joint_state_time).seconds() > joint_state_timeout_)
      {
        result->success        = false;
        result->final_position = current_pos;
        result->message        = "Goal aborted because the measured joint state is stale.";

        goal_handle->abort(result);

        // Hold at the last known position and mark the goal as no longer active.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_         = current_pos;
          has_active_goal_ = false;
        }

        return;
      }

      // If the goal has been active for too long without converging, abort with an appropriate
      // message.
      // This timeout might be confusing, so an explanation is worth including here:
      // if the goal is taking a long time to execute, it is likely that something has gone wrong
      // (e.g., the robot is stuck, the controller is not working, etc.) and it will never converge.
      // Without this timeout, the goal would run indefinitely and never report failure, which could be
      // dangerous and would require a manual shutdown to recover from.
      // By aborting after a reasonable amount of time, we allow the system to recover and try again
      // instead of getting stuck on a goal that will never succeed.
      if((now - goal_start_time).seconds() > goal_timeout_)
      {
        result->success        = false;
        result->final_position = current_pos;
        result->message        = "Goal aborted because it did not converge before goal_timeout.";

        goal_handle->abort(result);

        // Hold at the current position and mark the goal as no longer active.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_         = current_pos;
          has_active_goal_ = false;
        }

        return;
      }

      // CANCELLATION: Check if the client has requested cancellation. This is processed after
      // error conditions but before success, so critical errors take priority.
      if(goal_handle->is_canceling())
      {
        result->success        = false;
        result->final_position = current_pos;
        result->message        = "Goal canceled.";

        goal_handle->canceled(result);

        // At this point, we know current_pos_is_valid is true (error checks passed above),
        // so we can safely hold at current_pos and mark the goal as no longer active.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_         = current_pos;
          has_active_goal_ = false;
        }

        return;
      }

      // Convergence check: succeed if the measured position is within tolerance.
      if(std::abs(target_pos - current_pos) <= position_tolerance_)
      {
        result->success        = true;
        result->final_position = current_pos;
        result->message        = "Target fork position reached.";

        goal_handle->succeed(result);

        // Hold at the current position and mark the goal as no longer active.
        // The timer will continue publishing this position indefinitely, keeping the fork stable.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_         = current_pos;
          has_active_goal_ = false;
        }

        return;
      }

      // Update the commanded position for the timer to publish continuously.
      // The timer (pos_cmd_timer_cb) handles publication at command_publication_frequency.
      // Here, we simply update pos_cmd_ at the execution loop frequency, ensuring the timer
      // always publishes the most recent commanded position.
      {
        std::lock_guard<std::mutex> lock(mutex_);
        pos_cmd_ = target_pos;
      }

      if((now - last_feedback_time) >= rclcpp::Duration(feedback_period))
      {
        const auto feedback        = std::make_shared<ForkPosition::Feedback>();
        feedback->target_position  = target_pos;
        feedback->current_position = current_pos;
        feedback->position_error   = target_pos - current_pos;
        goal_handle->publish_feedback(feedback);
        last_feedback_time = now;
      }

      // Sleep for one execution loop period. This controls how often we check for convergence,
      // cancellation, timeouts, and joint state validity. Command and feedback publications are
      // decoupled from this loop frequency and publish at their own respective rates.
      std::this_thread::sleep_for(tick);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void ForkPositionControllerServer::handle_accepted(const std::shared_ptr<GoalHandleForkPosition> goal_handle)
  {
    RCLCPP_INFO(action_server_logger_,
                "Passing accepted goal %.6f to the execution logic.",
                goal_handle->get_goal()->position);

    // `execute()` is a long-running loop that must not block the executor. It is moved to a
    // detached thread so the executor remains free to dispatch joint_state_cb and future
    // action callbacks while the goal is in progress.
    //
    // `shared_from_this()` is used instead of capturing `this` directly. The detached thread
    // keeps its own shared_ptr to the node, which prevents the node from being destroyed while
    // `execute()` is still running. Without this, a node shutdown while a goal is active would
    // leave the thread with a dangling pointer and cause undefined behavior.
    //
    // `ForkPositionControllerServer` inherits `std::enable_shared_from_this` through
    // `rclcpp::Node`, so `shared_from_this()` is valid as long as the node was created via
    // `std::make_shared` (which is always the case with rclcpp nodes).
    auto self = std::static_pointer_cast<ForkPositionControllerServer>(shared_from_this());
    std::thread{[self, goal_handle]() {
      self->execute(goal_handle);
    }}.detach();
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  rclcpp_action::CancelResponse ForkPositionControllerServer::handle_cancel(
    const std::shared_ptr<GoalHandleForkPosition> /*goal_handle*/)
  {
    // Mark the active goal as canceled so the execute() thread can detect it and terminate early.
    // The execute() thread is responsible for setting has_active_goal_ back to false and calling the appropriate
    // goal handle method (canceled() or abort()) to end the goal lifecycle.
    RCLCPP_INFO(action_server_logger_, "Received cancel request.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  rclcpp_action::GoalResponse ForkPositionControllerServer::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                                        std::shared_ptr<const ForkPosition::Goal> goal)
  {
    // If the goal is outside the allowed range, reject it.
    if(goal->position < lower_limit_ || goal->position > upper_limit_)
    {
      RCLCPP_WARN(action_server_logger_,
                  "Rejecting goal %.6f because it is outside the allowed range [%.6f, %.6f].",
                  goal->position,
                  lower_limit_,
                  upper_limit_);
      return rclcpp_action::GoalResponse::REJECT;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);

      // If there is already an active goal, reject the new one.
      // This ensures only one goal is active at a time.
      // To accept a new goal, the client must first cancel the active one, which triggers the execute() thread to
      // terminate and
      if(has_active_goal_)
      {
        RCLCPP_WARN(action_server_logger_,
                    "Rejecting goal %.6f because another goal is already active.",
                    goal->position);
        return rclcpp_action::GoalResponse::REJECT;
      }

      // If the current position is not valid or the latest JointState is too old, reject the goal
      // to avoid executing it.
      if(!current_pos_is_valid_ || (this->now() - last_joint_state_time_).seconds() > joint_state_timeout_)
      {
        RCLCPP_WARN(action_server_logger_,
                    "Rejecting goal %.6f because no fresh JointState is available.",
                    goal->position);
        return rclcpp_action::GoalResponse::REJECT;
      }

      // If here, there is no active goal, the current position is valid, and the latest
      // JointState is not obsolete, therefore, accept the goal.

      has_active_goal_ = true;
    }

    RCLCPP_INFO(action_server_logger_, "Accepting goal %.6f.", goal->position);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void ForkPositionControllerServer::joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    // Find the needed joint in the JointState message by name.
    const auto joint_it = std::find(msg->name.begin(), msg->name.end(), joint_name_);

    // If the joint is not found, mark the current position as invalid and warn.
    if(joint_it == msg->name.end())
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_pos_          = std::numeric_limits<double>::quiet_NaN();
      current_pos_is_valid_ = false;

      RCLCPP_WARN_THROTTLE(joint_state_logger_,
                           *this->get_clock(),
                           1000,
                           "Joint '%s' not found in JointState message.",
                           joint_name_.c_str());
      return;
    }

    // Transform the iterator into an index to access the corresponding position entry.
    // If the index is out of bounds for the position vector, mark the current position as invalid and warn.
    const auto joint_index = static_cast<std::size_t>(std::distance(msg->name.begin(), joint_it));

    if(joint_index >= msg->position.size())
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_pos_          = std::numeric_limits<double>::quiet_NaN();
      current_pos_is_valid_ = false;

      RCLCPP_WARN_THROTTLE(joint_state_logger_,
                           *this->get_clock(),
                           1000,
                           "Joint '%s' has no position entry in JointState message.",
                           joint_name_.c_str());
      return;
    }

    // Update the current position with the received value under the mutex to ensure thread safety
    // with the execute() thread.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_pos_           = msg->position[joint_index];
      current_pos_is_valid_  = true;
      last_joint_state_time_ = this->now();
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void ForkPositionControllerServer::publish_pos_cmd(double position)
  {
    // If posicion is not defined, do not publish it and warn.
    if(std::isnan(position))
    {
      return;
    }

    // The command is expressed in meters. Round it to 0.001 m before publishing it.
    const double rounded_pos_cmd = std::round(position * 1000.0) / 1000.0;

    std_msgs::msg::Float64 command_msg;
    command_msg.data = rounded_pos_cmd;
    command_pub_->publish(command_msg);
  }
}  // namespace fork_position_controller_server
