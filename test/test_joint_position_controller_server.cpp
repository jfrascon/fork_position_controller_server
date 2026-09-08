#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "joint_position_controller_interfaces/action/joint_position.hpp"
#include "joint_position_controller_server/joint_position_controller_client.hpp"
#include "joint_position_controller_server/joint_position_controller_client_single_goal_sequential.hpp"
#include "joint_position_controller_server/joint_position_controller_server.hpp"

namespace
{
  using Server = joint_position_controller_server::JointPositionControllerServer;
  using Client = joint_position_controller_server::JointPositionControllerClient;
  using SequentialClient = joint_position_controller_server::JointPositionControllerClientSingleGoalSequential;
  using JointPosition = joint_position_controller_interfaces::action::JointPosition;

  rclcpp::NodeOptions make_server_options(const std::vector<rclcpp::Parameter>& additional_parameters = {})
  {
    std::vector<rclcpp::Parameter> parameters{
      rclcpp::Parameter{"lower_limit", 0.0},
      rclcpp::Parameter{"upper_limit", 1.0},
      rclcpp::Parameter{"joint_name", "test_joint"},
    };
    parameters.insert(parameters.end(), additional_parameters.begin(), additional_parameters.end());

    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    options.arguments({"--ros-args", "--remap", "__ns:=/joint_position_controller_server_test"});
    return options;
  }

  rclcpp::NodeOptions make_client_options(const std::vector<rclcpp::Parameter>& parameters)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(parameters);
    return options;
  }

  class JointPositionControllerServerTest: public ::testing::Test
  {
    protected:
      static void SetUpTestSuite()
      {
        rclcpp::init(0, nullptr);
      }

      static void TearDownTestSuite()
      {
        rclcpp::shutdown();
      }
  };

  TEST_F(JointPositionControllerServerTest, accepts_valid_parameters)
  {
    EXPECT_NO_THROW(std::make_shared<Server>(make_server_options()));
  }

  TEST_F(JointPositionControllerServerTest, rejects_non_finite_numeric_parameters)
  {
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THROW(std::make_shared<Server>(make_client_options({
                   rclcpp::Parameter{"lower_limit", nan},
                   rclcpp::Parameter{"upper_limit", 1.0},
                   rclcpp::Parameter{"joint_name", "test_joint"},
                 })),
                 std::invalid_argument);
    EXPECT_THROW(std::make_shared<Server>(make_client_options({
                   rclcpp::Parameter{"lower_limit", 0.0},
                   rclcpp::Parameter{"upper_limit", nan},
                   rclcpp::Parameter{"joint_name", "test_joint"},
                 })),
                 std::invalid_argument);

    const std::vector<std::string> parameter_names{
      "position_tolerance",
      "command_publication_frequency",
      "feedback_publication_frequency",
      "execution_loop_frequency",
      "goal_timeout",
      "joint_state_timeout",
    };

    for(const auto& parameter_name: parameter_names)
    {
      EXPECT_THROW(std::make_shared<Server>(make_server_options({rclcpp::Parameter{parameter_name, nan}})),
                   std::invalid_argument)
        << parameter_name;
    }
  }

  TEST_F(JointPositionControllerServerTest, rejects_non_finite_goal)
  {
    auto server = std::make_shared<Server>(make_server_options());
    auto client_node = std::make_shared<rclcpp::Node>("joint_position_test_client",
                                                      "/joint_position_controller_server_test");
    auto client = rclcpp_action::create_client<JointPosition>(client_node, "joint_position");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    executor.add_node(client_node);
    std::thread spin_thread{[&executor]() {
      executor.spin();
    }};

    const bool server_available = client->wait_for_action_server(std::chrono::seconds{2});
    JointPosition::Goal goal;
    goal.position = std::numeric_limits<double>::quiet_NaN();
    auto goal_future = client->async_send_goal(goal);
    const auto future_status = goal_future.wait_for(std::chrono::seconds{2});
    const auto goal_handle = future_status == std::future_status::ready ? goal_future.get() : nullptr;

    executor.cancel();
    spin_thread.join();

    ASSERT_TRUE(server_available);
    ASSERT_EQ(future_status, std::future_status::ready);
    EXPECT_EQ(goal_handle, nullptr);
  }

  TEST_F(JointPositionControllerServerTest, publishes_the_exact_requested_position)
  {
    auto server = std::make_shared<Server>(make_server_options());
    auto client_node = std::make_shared<rclcpp::Node>("joint_position_command_test_client",
                                                      "/joint_position_controller_server_test");
    auto action_client = rclcpp_action::create_client<JointPosition>(client_node, "joint_position");
    auto joint_state_publisher = client_node->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                                                             rclcpp::SensorDataQoS());

    std::promise<double> command_promise;
    auto command_future = command_promise.get_future();
    bool command_received = false;
    auto command_subscription = client_node->create_subscription<
      std_msgs::msg::Float64>("joint_commands/position",
                              10,
                              [&command_promise, &command_received](const std_msgs::msg::Float64& message) {
                                if(!command_received)
                                {
                                  command_received = true;
                                  command_promise.set_value(message.data);
                                }
                              });

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    executor.add_node(client_node);
    std::thread spin_thread{[&executor]() {
      executor.spin();
    }};

    const bool server_available = action_client->wait_for_action_server(std::chrono::seconds{2});

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = client_node->now();
    joint_state.name = {"test_joint"};
    joint_state.position = {0.0};
    for(int attempt = 0; attempt < 5; ++attempt)
    {
      joint_state_publisher->publish(joint_state);
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }

    constexpr double target_position = 0.123456;
    JointPosition::Goal goal;
    goal.position = target_position;
    auto goal_future = action_client->async_send_goal(goal);
    const auto goal_status = goal_future.wait_for(std::chrono::seconds{2});
    const auto goal_handle = goal_status == std::future_status::ready ? goal_future.get() : nullptr;
    const auto command_status = command_future.wait_for(std::chrono::seconds{2});
    const double published_position = command_status == std::future_status::ready ? command_future.get() : 0.0;

    if(goal_handle)
    {
      auto result_future = action_client->async_get_result(goal_handle);
      action_client->async_cancel_goal(goal_handle);
      result_future.wait_for(std::chrono::seconds{2});
    }

    executor.cancel();
    spin_thread.join();

    ASSERT_TRUE(server_available);
    ASSERT_EQ(goal_status, std::future_status::ready);
    ASSERT_NE(goal_handle, nullptr);
    ASSERT_EQ(command_status, std::future_status::ready);
    EXPECT_DOUBLE_EQ(published_position, target_position);
    (void)command_subscription;
  }

  TEST_F(JointPositionControllerServerTest, ignores_joint_state_messages_for_other_joints)
  {
    auto server = std::make_shared<Server>(make_server_options());
    auto client_node = std::make_shared<rclcpp::Node>("joint_position_state_test_client",
                                                      "/joint_position_controller_server_test");
    auto action_client = rclcpp_action::create_client<JointPosition>(client_node, "joint_position");
    auto joint_state_publisher = client_node->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                                                             rclcpp::SensorDataQoS());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server);
    executor.add_node(client_node);
    std::thread spin_thread{[&executor]() {
      executor.spin();
    }};

    const bool server_available = action_client->wait_for_action_server(std::chrono::seconds{2});

    sensor_msgs::msg::JointState controlled_joint_state;
    controlled_joint_state.header.stamp = client_node->now();
    controlled_joint_state.name = {"test_joint"};
    controlled_joint_state.position = {0.4};
    for(int attempt = 0; attempt < 3; ++attempt)
    {
      joint_state_publisher->publish(controlled_joint_state);
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }

    sensor_msgs::msg::JointState other_joint_state;
    other_joint_state.header.stamp = client_node->now();
    other_joint_state.name = {"other_joint"};
    other_joint_state.position = {0.1};
    for(int attempt = 0; attempt < 3; ++attempt)
    {
      joint_state_publisher->publish(other_joint_state);
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }

    JointPosition::Goal goal;
    goal.position = 0.4;
    auto goal_future = action_client->async_send_goal(goal);
    const auto goal_status = goal_future.wait_for(std::chrono::seconds{2});
    const auto goal_handle = goal_status == std::future_status::ready ? goal_future.get() : nullptr;

    rclcpp_action::ClientGoalHandle<JointPosition>::WrappedResult result;
    auto result_status = std::future_status::timeout;
    if(goal_handle)
    {
      auto result_future = action_client->async_get_result(goal_handle);
      result_status = result_future.wait_for(std::chrono::seconds{2});
      if(result_status == std::future_status::ready)
      {
        result = result_future.get();
      }
    }

    executor.cancel();
    spin_thread.join();

    ASSERT_TRUE(server_available);
    ASSERT_EQ(goal_status, std::future_status::ready);
    ASSERT_NE(goal_handle, nullptr);
    ASSERT_EQ(result_status, std::future_status::ready);
    ASSERT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
    ASSERT_NE(result.result, nullptr);
    EXPECT_TRUE(result.result->success);
  }

  TEST_F(JointPositionControllerServerTest, clients_reject_invalid_parameters)
  {
    const auto nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_THROW(std::make_shared<Client>(make_client_options({rclcpp::Parameter{"position", nan}})),
                 std::invalid_argument);
    EXPECT_THROW(std::make_shared<Client>(make_client_options({rclcpp::Parameter{"wait_for_server_timeout", 0.0}})),
                 std::invalid_argument);
    EXPECT_THROW(std::make_shared<SequentialClient>(
                   make_client_options({rclcpp::Parameter{"second_goal_delay", -1.0}})),
                 std::invalid_argument);
    EXPECT_THROW(std::make_shared<SequentialClient>(make_client_options({rclcpp::Parameter{"second_position", nan}})),
                 std::invalid_argument);
  }
}  // namespace
