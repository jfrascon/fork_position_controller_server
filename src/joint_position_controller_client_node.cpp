#include "joint_position_controller_server/joint_position_controller_client.hpp"

#include <chrono>
#include <exception>
#include <memory>

#include <rclcpp/executors/single_threaded_executor.hpp>

/**
 * @brief Run the single-goal example client node.
 * @param argc Command-line argument count.
 * @param argv Command-line argument vector.
 * @return Process return code.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<joint_position_controller_server::JointPositionControllerClient> node;
  int ret{0};

  try
  {
    node = std::make_shared<joint_position_controller_server::JointPositionControllerClient>();

    if(!node->client()->wait_for_action_server(std::chrono::duration<double>(node->wait_for_server_timeout())))
    {
      RCLCPP_FATAL(node->get_logger(),
                   "Action server '%s' not available after %.2f seconds.",
                   node->action_name().c_str(),
                   node->wait_for_server_timeout());
      ret = 1;
    }
    else
    {
      // The single-goal example waits synchronously for acceptance and then waits synchronously for
      // the terminal result of that same goal.
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);

      auto goal_options = node->create_send_goal_options();
      auto goal_future = node->client()->async_send_goal(node->goal_from_parameters(), goal_options);

      if(executor.spin_until_future_complete(goal_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_FATAL(node->get_logger(), "Failed to send goal to the action server.");
        ret = 1;
      }
      else
      {
        auto goal_handle = goal_future.get();

        if(!goal_handle)
        {
          RCLCPP_WARN(node->get_logger(), "Goal rejected by the action server.");
          ret = 1;
        }
        else
        {
          auto result_future = node->client()->async_get_result(goal_handle);

          if(executor.spin_until_future_complete(result_future) != rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_FATAL(node->get_logger(), "Failed to receive the final goal result.");
            ret = 1;
          }
          else
          {
            const auto result = result_future.get();
            if(result.code != rclcpp_action::ResultCode::SUCCEEDED || !result.result || !result.result->success)
            {
              ret = 1;
            }
          }
        }
      }
    }
  }
  catch(const std::exception& ex)
  {
    const auto logger = node ? node->get_logger() : rclcpp::get_logger("joint_position_controller_client_node");
    RCLCPP_FATAL(logger, "%s.", ex.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
