#include "joint_position_controller_server/joint_position_controller_server.hpp"

#include <exception>
#include <memory>

#include <rclcpp/executors/single_threaded_executor.hpp>

/**
 * @brief Run the joint position controller server node.
 * @param argc Command-line argument count.
 * @param argv Command-line argument vector.
 * @return Process return code.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<joint_position_controller_server::JointPositionControllerServer> node;
  int ret{0};

  try
  {
    node = std::make_shared<joint_position_controller_server::JointPositionControllerServer>();

    // All ROS callbacks in this node (joint_state_cb, handle_goal, handle_cancel,
    // handle_accepted) use the node's default callback group, which is MutuallyExclusive.
    // They are therefore sequential and a single executor thread is sufficient.
    // Goal execution runs in a separate detached thread that lives outside the executor,
    // so it never blocks ROS callbacks.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  }
  catch(const std::exception& ex)
  {
    const auto logger = node ? node->get_logger() : rclcpp::get_logger("joint_position_controller_server_node");
    RCLCPP_FATAL(logger, "%s.", ex.what());
    ret = 1;
  }

  rclcpp::shutdown();
  return ret;
}
