/**
 * @file talker.cpp
 * @brief ROS 2 C++ publisher for the Hello World example.
 *
 * This node publishes std_msgs/msg/String messages on the relative topic
 * `hello`. With the default namespace, the topic is visible as `/hello`.
 * A wall timer invokes the publishing callback once per second.
 */

#include <chrono>
#include <cstdint>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class HelloWorldPublisher
 * @brief Publishes a numbered greeting on the `hello` topic.
 *
 * The class owns the ROS publisher, the periodic timer, and the message
 * counter. Keeping these resources as members ensures they remain alive for
 * the whole lifetime of the node while the executor is spinning.
 */
class HelloWorldPublisher : public rclcpp::Node
{
public:
  /**
   * @brief Construct the publisher node and configure its ROS interfaces.
   *
   * The node is registered as `hello_world_talker`. The publisher uses a
   * queue depth of 10 and the timer period is one second.
   */
  HelloWorldPublisher()
  : Node("hello_world_talker"), count_(0)
  {
    // Advertise String messages on the relative `hello` topic. The queue
    // depth controls how many outgoing messages can wait for subscribers.
    publisher_ = this->create_publisher<std_msgs::msg::String>("hello", 10);

    // Let the ROS executor invoke publish_message() every second. A timer
    // keeps the callback non-blocking and integrates with normal ROS spinning.
    timer_ = this->create_wall_timer(
      1s, std::bind(&HelloWorldPublisher::publish_message, this));
  }

private:
  /**
   * @brief Build and publish the next greeting message.
   *
   * The current counter value is placed in the message before publishing.
   * The counter is then incremented so the next invocation receives the next
   * number. The same payload is logged locally for terminal feedback.
   */
  void publish_message()
  {
    // Construct a fresh ROS message for this timer event. The `data` field
    // contains the text that subscribers receive.
    std_msgs::msg::String message;
    message.data = "Hello from ROS 2! Message " + std::to_string(count_);

    // Send the message to every compatible subscriber on `hello`.
    publisher_->publish(message);

    // Log the exact payload so the publisher is observable without a second
    // terminal or a running subscriber.
    RCLCPP_INFO(this->get_logger(), "%s", message.data.c_str());

    // Increment after publishing so the logged and transmitted number match.
    ++count_;
  }

  // Publisher handle retained for the lifetime of the node.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // Timer handle retained so the callback remains scheduled while spinning.
  rclcpp::TimerBase::SharedPtr timer_;

  // Number included in the next message; the first message is numbered zero.
  std::uint64_t count_;
};

/**
 * @brief Start and run the Hello World publisher.
 *
 * The ROS client library is initialized before constructing the node. The
 * executor then processes timer events until shutdown, after which the node
 * is released and the ROS client library is shut down cleanly.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line argument values passed to rclcpp.
 * @return EXIT_SUCCESS after the node stops normally.
 */
int main(int argc, char * argv[])
{
  // Initialize ROS before creating nodes or other ROS entities.
  rclcpp::init(argc, argv);

  // Create the node, including its publisher and periodic timer.
  auto node = std::make_shared<HelloWorldPublisher>();

  // The executor waits for timer events and dispatches publish_message().
  rclcpp::spin(node);

  // Release ROS resources after spin returns, including after Ctrl+C.
  rclcpp::shutdown();
  return 0;
}
