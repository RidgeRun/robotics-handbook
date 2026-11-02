"""ROS 2 subscriber node for the Hello World example.

This module contains a minimal subscriber that listens for
``std_msgs/msg/String`` messages on the relative topic ``hello``. For the
default namespace, the topic is exposed as ``/hello``. Every received message
is written to the node logger, making it easy to verify communication with the
publisher from a terminal.

The executable is intentionally focused on the subscriber lifecycle: create a
subscription, spin the node so callbacks can run, and release ROS resources
when the process is interrupted.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldSubscriber(Node):
    """Print each greeting received from the ``hello`` topic.

    The subscription uses ``std_msgs/msg/String`` and a queue depth of 10 to
    match :class:`helloworld.talker.HelloWorldPublisher`. ROS 2 invokes
    :meth:`on_message` whenever a compatible message arrives.

    Attributes:
        subscription (rclpy.subscription.Subscription): Subscription handle
            retained by the node for the lifetime of the subscriber. Keeping
            this handle as an instance attribute ensures the subscription
            remains alive while the node is spinning.
    """

    def __init__(self):
        """Initialize the node and subscribe to the ``hello`` topic.

        The ROS node is registered with the name ``hello_world_listener``.
        The callback is registered with a queue depth of 10, allowing the
        executor to buffer a small number of incoming messages when callback
        execution temporarily lags behind message arrival.

        Returns:
            None: The initialized node instance is configured in place.
        """
        # Register this object with the ROS graph under a descriptive node
        # name. The name is visible in tools such as `ros2 node list`.
        super().__init__('hello_world_listener')

        # Subscribe to the same message type and topic used by the talker.
        # ROS 2 will call `self.on_message` whenever a message is received.
        self.subscription = self.create_subscription(
            String,
            'hello',
            self.on_message,
            10,
        )

        # Keep the subscription handle as an instance attribute for the full
        # lifetime of the node so the subscription remains active while the
        # executor is spinning.

    def on_message(self, message):
        """Log one message received from the ``hello`` topic.

        This callback is called by the ROS 2 executor with a populated
        ``std_msgs/msg/String`` instance. It does not modify or republish the
        message; it simply prefixes the payload with ``Received:`` and sends
        it to the node logger.

        Args:
            message (std_msgs.msg.String): Incoming message whose ``data``
                field contains the greeting text published by the talker.

        Returns:
            None: Subscriber callbacks communicate their result through the
            logger rather than a return value.
        """
        # The callback receives a `std_msgs/msg/String` object. Read its data
        # field rather than printing the message object representation.
        self.get_logger().info(f'Received: {message.data}')


def main(args=None):
    """Run the Hello World subscriber until the process is interrupted.

    This function is the console-script entry point installed as
    ``ros2 run helloworld listener``. It initializes ``rclpy`` with the
    command line arguments supplied by ROS 2, creates the subscriber node, and
    calls :func:`rclpy.spin` so incoming messages can trigger
    :meth:`HelloWorldSubscriber.on_message`. A ``KeyboardInterrupt`` is
    treated as a normal shutdown, and both the node and ROS client library are
    cleaned up in the ``finally`` block.

    Args:
        args (list[str] | None): Optional command line arguments passed to
            :func:`rclpy.init`. ``None`` makes ``rclpy`` use the process
            command line.

    Returns:
        None: The function returns after the node has been shut down.
    """
    # Initialize the ROS client library before creating the subscriber node.
    rclpy.init(args=args)

    # Construct the node and register its `hello` subscription.
    node = HelloWorldSubscriber()
    try:
        # Wait for incoming messages and dispatch each one to `on_message`.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Ctrl+C is the normal way to stop this example, so exit quietly.
        pass
    finally:
        # Release ROS resources in both normal and interrupted shutdown
        # paths. This keeps the process lifecycle well-defined.
        node.destroy_node()
        rclpy.shutdown()
