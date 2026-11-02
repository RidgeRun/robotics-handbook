/**
 * @file listener.cpp
 * @brief ROS 2 C++ subscriber for the Hello World example.
 *
 * This node subscribes to std_msgs/msg/String messages on the relative topic
 * `hello`. Each received message is printed through the ROS logger.
 */

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class HelloWorldSubscriber
 * @brief Logs each greeting received from the `hello` topic.
 *
 * The subscription handle is stored as a member so it remains active for the
 * complete lifetime of the node while the executor dispatches callbacks.
 */
class HelloWorldSubscriber : public rclcpp::Node
{
public:
  /**
   * @brief Construct the subscriber node and register its subscription.
   *
   * The node is registered as `hello_world_listener`. The callback uses the
   * same message type, topic, and queue depth as the talker.
   */
  HelloWorldSubscriber()
  : Node("hello_world_listener")
  {
    // Subscribe to String messages on `hello`. ROS invokes on_message() when
    // a compatible message arrives, using a queue depth of 10.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "hello",
      10,
      std::bind(&HelloWorldSubscriber::on_message, this, std::placeholders::_1));
  }

private:
  /**
   * @brief Handle one incoming greeting message.
   *
   * The callback only observes the message and logs its `data` field; it does
   * not modify or republish the received message.
   *
   * @param message Shared pointer to the received String message.
   */
  void on_message(const std_msgs::msg::String::SharedPtr message) const
  {
    // Read the payload from the typed message and print it through the node's
    // logger instead of printing the C++ object representation.
    RCLCPP_INFO(this->get_logger(), "Received: %s", message->data.c_str());
  }

  // Subscription handle retained for the lifetime of the node.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Start and run the Hello World subscriber.
 *
 * The executor waits for incoming messages and dispatches on_message(). When
 * spinning ends, the node and ROS client library are shut down cleanly.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line argument values passed to rclcpp.
 * @return EXIT_SUCCESS after the node stops normally.
 */
int main(int argc, char * argv[])
{
  // Initialize ROS before creating nodes or subscriptions.
  rclcpp::init(argc, argv);

  // Create the node and register its `hello` subscription.
  auto node = std::make_shared<HelloWorldSubscriber>();

  // Wait for incoming messages and dispatch each one to on_message().
  rclcpp::spin(node);

  // Release ROS resources after spin returns, including after Ctrl+C.
  rclcpp::shutdown();
  return 0;
}
