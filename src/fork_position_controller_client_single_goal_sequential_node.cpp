#include "fork_position_controller_server/fork_position_controller_client_single_goal_sequential.hpp"

#include <chrono>
#include <exception>
#include <memory>

#include <rclcpp/executors/single_threaded_executor.hpp>

/**
 * @brief Run the sequential single-goal example client node.
 * @param argc Command-line argument count.
 * @param argv Command-line argument vector.
 * @return Process return code.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<fork_position_controller_server::ForkPositionControllerClientSingleGoalSequential> node;
  int ret{0};

  try
  {
    node = std::make_shared<fork_position_controller_server::ForkPositionControllerClientSingleGoalSequential>();

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
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);

      node->start();

      if(executor.spin_until_future_complete(node->completion_future()) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_FATAL(node->get_logger(), "Sequential example did not finish cleanly.");
        ret = 1;
      }
      else
      {
        ret = node->completion_future().get();
      }
    }
  }
  catch(const std::exception& ex)
  {
    const auto logger = node ? node->get_logger() :
                               rclcpp::get_logger("fork_position_controller_client_single_goal_sequential_node");
    RCLCPP_FATAL(logger, "%s.", ex.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
