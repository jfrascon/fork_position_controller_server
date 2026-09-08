#include "joint_position_controller_server/joint_position_controller_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

namespace joint_position_controller_server
{
  JointPositionControllerServer::JointPositionControllerServer(const rclcpp::NodeOptions& options):
    rclcpp::Node("joint_position_controller_server", options),
    action_server_logger_(this->get_logger().get_child("action_server")),
    joint_state_logger_(this->get_logger().get_child("joint_state_cb"))
  {
    declare_and_validate_parameters();
    create_subs_pubs();

    RCLCPP_INFO(this->get_logger(), "JointPositionControllerServer node initialized.");
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void JointPositionControllerServer::create_subs_pubs()
  {
    command_pub_ = this->create_publisher<std_msgs::msg::Float64>(command_topic_, 10);

    // All callbacks (joint_state_cb, handle_goal, handle_cancel, handle_accepted) use the
    // node's default MutuallyExclusive callback group, so no explicit group is needed.
    // Sharing the same MutuallyExclusive group guarantees they never run concurrently.
    joint_states_sub_ = this->create_subscription<
      sensor_msgs::msg::JointState>(joint_states_topic_,
                                    rclcpp::SensorDataQoS(),
                                    std::bind(&JointPositionControllerServer::joint_state_cb,
                                              this,
                                              std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<
      JointPosition>(this,
                     action_name_,
                     std::bind(&JointPositionControllerServer::handle_goal,
                               this,
                               std::placeholders::_1,
                               std::placeholders::_2),
                     std::bind(&JointPositionControllerServer::handle_cancel, this, std::placeholders::_1),
                     std::bind(&JointPositionControllerServer::handle_accepted, this, std::placeholders::_1));

    // Create a timer that publishes position commands at the configured frequency.
    // This ensures continuous publication even after execute() terminates, maintaining
    // the joint in its target/holding position.
    const auto timer_period = std::chrono::duration<double>(1.0 / command_publication_frequency_);
    pos_cmd_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
                                             std::bind(&JointPositionControllerServer::pos_cmd_timer_cb, this));
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void JointPositionControllerServer::declare_and_validate_parameters()
  {
    // Declare parameters before reading them so launch files can override the backend topics,
    // joint name, limits, and timing policy through normal ROS 2 parameter injection.
    // The node intentionally keeps these values fixed after construction. Runtime changes would
    // need extra synchronization with the action execution thread and command timer.
    this->declare_parameter<double>("lower_limit");
    this->declare_parameter<double>("upper_limit");
    this->declare_parameter<double>("position_tolerance", 0.005);
    this->declare_parameter<double>("command_publication_frequency", 20.0);
    this->declare_parameter<double>("feedback_publication_frequency", 10.0);
    this->declare_parameter<double>("execution_loop_frequency", 30.0);
    this->declare_parameter<double>("goal_timeout", 10.0);
    this->declare_parameter<double>("joint_state_timeout", 1.0);
    this->declare_parameter<std::string>("joint_name");
    this->declare_parameter<std::string>("command_topic", "joint_commands/position");
    this->declare_parameter<std::string>("joint_states_topic", "joint_states");

    command_topic_ = this->get_parameter("command_topic").as_string();
    joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();
    joint_name_ = this->get_parameter("joint_name").as_string();
    lower_limit_ = this->get_parameter("lower_limit").get_value<double>();
    upper_limit_ = this->get_parameter("upper_limit").get_value<double>();
    position_tolerance_ = this->get_parameter("position_tolerance").get_value<double>();
    command_publication_frequency_ = this->get_parameter("command_publication_frequency").get_value<double>();
    feedback_publication_frequency_ = this->get_parameter("feedback_publication_frequency").get_value<double>();
    execution_loop_frequency_ = this->get_parameter("execution_loop_frequency").get_value<double>();
    goal_timeout_ = this->get_parameter("goal_timeout").get_value<double>();
    joint_state_timeout_ = this->get_parameter("joint_state_timeout").get_value<double>();

    // Reject NaN and infinities early. These parameters feed comparisons, timer periods, and
    // published command values, so accepting a non-finite value would make later checks unreliable.
    const auto require_finite_parameter = [](double value, const std::string& parameter_name) {
      if(!std::isfinite(value))
      {
        throw std::invalid_argument("Parameter '" + parameter_name + "' must be finite.");
      }
    };

    require_finite_parameter(lower_limit_, "lower_limit");
    require_finite_parameter(upper_limit_, "upper_limit");
    require_finite_parameter(position_tolerance_, "position_tolerance");
    require_finite_parameter(command_publication_frequency_, "command_publication_frequency");
    require_finite_parameter(feedback_publication_frequency_, "feedback_publication_frequency");
    require_finite_parameter(execution_loop_frequency_, "execution_loop_frequency");
    require_finite_parameter(goal_timeout_, "goal_timeout");
    require_finite_parameter(joint_state_timeout_, "joint_state_timeout");

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

  void JointPositionControllerServer::pos_cmd_timer_cb()
  {
    // Read the commanded position under the mutex and then publish outside the critical section.
    // This keeps the lock hold time short and avoids calling ROS publisher code while the shared
    // action state is locked.
    // publish_pos_cmd() treats NaN as "no active command", so startup and idle periods do not
    // publish stale or arbitrary positions.
    double cmd;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cmd = pos_cmd_;
    }

    publish_pos_cmd(cmd);
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void JointPositionControllerServer::execute(const std::shared_ptr<GoalHandleJointPosition> goal_handle)
  {
    const auto target_pos{goal_handle->get_goal()->position};

    // Capture the goal start time locally. The timeout is measured against the node clock so it
    // follows the same time source as the incoming JointState freshness checks.
    const auto goal_start_time{this->now()};

    // Feedback has its own publication period. It can be slower than the execution loop so clients
    // get useful progress updates without forcing every convergence check to publish feedback.
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

    const auto result{std::make_shared<JointPosition::Result>()};

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
        current_pos = current_pos_;
        current_pos_is_valid = current_pos_is_valid_;
        last_joint_state_time = last_joint_state_time_;
      }

      // Error conditions are checked before cancellation or success. If the server can no longer
      // prove what the joint is doing, aborting gives the client a stronger signal than reporting
      // a clean cancel or a stale success.

      // An invalid joint state prevents safe convergence checks and command publication.
      if(!current_pos_is_valid)
      {
        result->success = false;
        result->final_position = std::numeric_limits<double>::quiet_NaN();
        result->message = "Goal aborted because the joint state is invalid.";

        // Release the internal goal slot before publishing the terminal action result. A terminal
        // result can trigger client callbacks immediately, and those callbacks may submit another
        // goal. Clearing the slot first prevents a false "goal already active" rejection.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_ = std::numeric_limits<double>::quiet_NaN();
          has_active_goal_ = false;
        }

        goal_handle->abort(result);
        return;
      }

      // A stale measurement cannot prove that the joint is following the requested command.
      if((now - last_joint_state_time).seconds() > joint_state_timeout_)
      {
        result->success = false;
        result->final_position = current_pos;
        result->message = "Goal aborted because the measured joint state is stale.";

        // Keep publishing the last measured position as a holding command after aborting. This is
        // safer than keeping the unreachable target command when fresh feedback is unavailable.
        // Clear the active-goal slot before notifying the client, as above.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_ = current_pos;
          has_active_goal_ = false;
        }

        goal_handle->abort(result);
        return;
      }

      // The goal timeout lets clients recover when a blocked joint never converges.
      if((now - goal_start_time).seconds() > goal_timeout_)
      {
        result->success = false;
        result->final_position = current_pos;
        result->message = "Goal aborted because it did not converge before goal_timeout.";

        // The joint did not reach the target in time. Hold the last measured position instead of
        // continuing to push the expired target, then free the single-goal slot.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_ = current_pos;
          has_active_goal_ = false;
        }

        goal_handle->abort(result);
        return;
      }

      // Process cancellation after system failures but before reporting convergence.
      if(goal_handle->is_canceling())
      {
        result->success = false;
        result->final_position = current_pos;
        result->message = "Goal canceled.";

        // Cancelation stops chasing the target but keeps the last measured position as the hold
        // command. This leaves the backend in a known state after the action is canceled.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_ = current_pos;
          has_active_goal_ = false;
        }

        goal_handle->canceled(result);
        return;
      }

      // Succeed when the measured position is within the configured absolute tolerance.
      if(std::abs(target_pos - current_pos) <= position_tolerance_)
      {
        result->success = true;
        result->final_position = current_pos;
        result->message = "Target joint position reached.";

        // The timer continues publishing this measured position as the holding command. Use the
        // measured value instead of the target because it reflects where the joint actually ended.
        // Release the internal goal slot before publishing the terminal action result.
        {
          std::lock_guard<std::mutex> lock(mutex_);
          pos_cmd_ = current_pos;
          has_active_goal_ = false;
        }

        goal_handle->succeed(result);
        return;
      }

      // Update the target used by the independent command-publication timer. The execute loop does
      // not publish commands directly; it only chooses what the timer should publish. This keeps
      // command publication frequency stable even when feedback or goal checks take longer.
      {
        std::lock_guard<std::mutex> lock(mutex_);
        pos_cmd_ = target_pos;
      }

      // Publish feedback at its configured rate using the same stable position snapshot used for
      // convergence checks in this iteration.
      if((now - last_feedback_time) >= rclcpp::Duration(feedback_period))
      {
        const auto feedback = std::make_shared<JointPosition::Feedback>();
        feedback->target_position = target_pos;
        feedback->current_position = current_pos;
        feedback->position_error = target_pos - current_pos;
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

  void JointPositionControllerServer::handle_accepted(const std::shared_ptr<GoalHandleJointPosition> goal_handle)
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
    // `JointPositionControllerServer` inherits `std::enable_shared_from_this` through
    // `rclcpp::Node`, so `shared_from_this()` is valid as long as the node was created via
    // `std::make_shared` (which is always the case with rclcpp nodes).
    auto self = std::static_pointer_cast<JointPositionControllerServer>(shared_from_this());
    std::thread{[self, goal_handle]() {
      self->execute(goal_handle);
    }}.detach();
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  rclcpp_action::CancelResponse JointPositionControllerServer::handle_cancel(
    const std::shared_ptr<GoalHandleJointPosition> /*goal_handle*/)
  {
    // Accepting the request moves the goal handle into its canceling state.
    // The execution thread observes that state and publishes the terminal canceled result.
    RCLCPP_INFO(action_server_logger_, "Received cancel request.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  rclcpp_action::GoalResponse JointPositionControllerServer::handle_goal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const JointPosition::Goal> goal)
  {
    if(!std::isfinite(goal->position))
    {
      RCLCPP_WARN(action_server_logger_, "Rejecting goal because its position is not finite.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Reject targets outside the configured hard limits before reserving the single-goal slot.
    // The backend command topic should never receive positions outside this package boundary.
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

      // Reject concurrent goals because this server owns only one backend command stream.
      // A client must wait for the active goal to terminate before submitting another one.
      if(has_active_goal_)
      {
        RCLCPP_WARN(action_server_logger_,
                    "Rejecting goal %.6f because another goal is already active.",
                    goal->position);
        return rclcpp_action::GoalResponse::REJECT;
      }

      // Refuse to start without a fresh measurement. Otherwise the first command could be sent
      // while the node has no evidence that the backend joint exists or is reporting valid data.
      if(!current_pos_is_valid_ || (this->now() - last_joint_state_time_).seconds() > joint_state_timeout_)
      {
        RCLCPP_WARN(action_server_logger_,
                    "Rejecting goal %.6f because no fresh JointState is available.",
                    goal->position);
        return rclcpp_action::GoalResponse::REJECT;
      }

      // From this point until execute() clears the flag, the server owns the only accepted goal.
      // Reserve the slot in handle_goal(), not handle_accepted(), because ROS 2 calls those
      // callbacks separately and a second request could otherwise arrive between them.

      has_active_goal_ = true;
    }

    RCLCPP_INFO(action_server_logger_, "Accepting goal %.6f.", goal->position);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void JointPositionControllerServer::joint_state_cb(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
  {
    // Find the needed joint in the JointState message by name.
    const auto joint_it = std::find(msg->name.begin(), msg->name.end(), joint_name_);

    // Ignore JointState messages for other joints.
    // The stale-state timeout detects when this joint stops receiving its own measurements.
    if(joint_it == msg->name.end())
    {
      RCLCPP_WARN_THROTTLE(joint_state_logger_,
                           *this->get_clock(),
                           1000,
                           "Ignoring JointState message without joint '%s'.",
                           joint_name_.c_str());
      return;
    }

    // Convert the name iterator to the index of its matching position entry.
    // A missing position makes this JointState invalid for the configured joint.
    const auto joint_index = static_cast<std::size_t>(std::distance(msg->name.begin(), joint_it));

    if(joint_index >= msg->position.size())
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_pos_ = std::numeric_limits<double>::quiet_NaN();
      current_pos_is_valid_ = false;

      RCLCPP_WARN_THROTTLE(joint_state_logger_,
                           *this->get_clock(),
                           1000,
                           "Joint '%s' has no position entry in JointState message.",
                           joint_name_.c_str());
      return;
    }

    const double position = msg->position[joint_index];

    if(!std::isfinite(position))
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_pos_ = std::numeric_limits<double>::quiet_NaN();
      current_pos_is_valid_ = false;

      RCLCPP_WARN_THROTTLE(joint_state_logger_,
                           *this->get_clock(),
                           1000,
                           "Joint '%s' has a non-finite position in the JointState message.",
                           joint_name_.c_str());
      return;
    }

    // Update the current position under the mutex so the execution thread reads consistent data.
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_pos_ = position;
      current_pos_is_valid_ = true;
      last_joint_state_time_ = this->now();
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  void JointPositionControllerServer::publish_pos_cmd(double position)
  {
    // NaN is the internal sentinel for "no command to publish". Keeping that sentinel inside this
    // helper makes all callers safe, including the periodic timer during startup and idle states.
    if(!std::isfinite(position))
    {
      return;
    }

    std_msgs::msg::Float64 command_msg;
    command_msg.data = position;
    command_pub_->publish(command_msg);
  }
}  // namespace joint_position_controller_server
