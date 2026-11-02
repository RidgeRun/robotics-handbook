"""ROS 2 publisher node for the Hello World example.

This module contains a minimal publisher that demonstrates the basic ROS 2
Python node lifecycle:

* initialize the ROS client library;
* create a :class:`HelloWorldPublisher` node;
* publish messages from a timer callback; and
* spin until the process is interrupted before cleaning up.

The node publishes ``std_msgs/msg/String`` messages on the relative topic
``hello``. With the default namespace, this appears as the ``/hello`` topic.
Messages are published once per second and include a monotonically increasing
counter so that the flow of messages is easy to observe.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldPublisher(Node):
    """Publish a numbered greeting on the ``hello`` topic.

    The publisher uses a ROS 2 timer rather than a blocking loop. This lets
    the executor handle timer events and other ROS callbacks in the normal
    way. The node has no configurable ROS parameters in this small example;
    the topic name, message type, queue depth, and period are intentionally
    kept in the constructor so the example is easy to read.

    Attributes:
        publisher (rclpy.publisher.Publisher): Publisher for
            ``std_msgs/msg/String`` messages on the relative ``hello`` topic.
        count (int): Number placed in the next message. It starts at zero and
            increases by one after every successful publish operation.
        timer (rclpy.timer.Timer): Timer that invokes :meth:`publish_message`
            every one second.
    """

    def __init__(self):
        """Initialize the node, publisher, counter, and periodic timer.

        The ROS node is registered with the name ``hello_world_talker``. The
        publisher uses a queue depth of 10, which allows a small number of
        messages to wait if a subscriber temporarily cannot receive them.

        Returns:
            None: The initialized node instance is configured in place.
        """
        # Register this object with the ROS graph under a stable, descriptive
        # node name. The name is visible in tools such as `ros2 node list`.
        super().__init__('hello_world_talker')

        # Advertise a publisher for String messages. The topic is relative to
        # the node namespace, so it resolves to `/hello` by default. The
        # depth of 10 is the outgoing queue size used by this demo.
        self.publisher = self.create_publisher(String, 'hello', 10)

        # The first message is numbered zero. Keeping the counter on the node
        # makes its value persist between timer callback invocations.
        self.count = 0

        # Ask the ROS executor to call `publish_message` every second. A timer
        # avoids blocking the executor with a manual `while`/`sleep` loop.
        self.timer = self.create_timer(1.0, self.publish_message)

    def publish_message(self):
        """Create and publish the next greeting message.

        A new ``std_msgs/msg/String`` message is created with text in the
        format ``Hello from ROS 2! Message N``, where ``N`` is the current
        value of :attr:`count`. The message is published on ``hello`` and
        echoed through the node logger for visibility in the terminal. The
        counter is incremented after the publish call so the next timer event
        receives the next number.

        This method is used as a timer callback and should not be called from
        a separate thread in normal operation.

        Returns:
            None: The message is sent through the ROS publisher and no value
            is returned to the timer.
        """
        # Construct a fresh ROS message for this timer event. `String.data`
        # is the payload that subscribers receive.
        message = String()
        message.data = f'Hello from ROS 2! Message {self.count}'

        # Send the message to all compatible subscribers on the `hello` topic.
        self.publisher.publish(message)

        # Log the exact payload as local feedback, even when no subscriber is
        # running in another process.
        self.get_logger().info(message.data)

        # Increment only after publishing so the displayed number matches the
        # number assigned to the message that was just sent.
        self.count += 1


def main(args=None):
    """Run the Hello World publisher until the process is interrupted.

    This function is the console-script entry point installed as
    ``ros2 run helloworld talker``. It initializes ``rclpy`` with the command
    line arguments supplied by ROS 2, constructs the publisher node, and hands
    control to :func:`rclpy.spin` so timer callbacks can execute. A
    ``KeyboardInterrupt`` is treated as a normal shutdown, and the node and
    ROS client library are cleaned up in the ``finally`` block.

    Args:
        args (list[str] | None): Optional command line arguments passed to
            :func:`rclpy.init`. ``None`` makes ``rclpy`` use the process
            command line.

    Returns:
        None: The function returns after the node has been shut down.
    """
    # Initialize the ROS client library before creating nodes or using ROS
    # entities. `args=None` lets rclpy process the command line itself.
    rclpy.init(args=args)

    # Create the node, including its publisher and timer configuration.
    node = HelloWorldPublisher()
    try:
        # Hand control to the executor. It waits for timer events and invokes
        # `publish_message` once per second until shutdown is requested.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C is an expected way to stop this demo, not an application
        # error, so no error message is required here.
        pass
    finally:
        # Always release the node and terminate the ROS client library, even
        # when spinning ends because of an interrupt or another shutdown path.
        node.destroy_node()
        rclpy.shutdown()
