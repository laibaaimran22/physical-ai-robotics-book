---
sidebar_position: 2
title: "ROS 2 Nodes, Topics, and Services in Practice"
---

import ChapterTranslator from '@site/src/components/Translation/ChapterTranslator';

<ChapterTranslator>

# ROS 2 Nodes, Topics, and Services in Practice

## Overview
This lesson dives into practical implementation of ROS 2 nodes, understanding how to create them, publish and subscribe to topics for data exchange, and implement services for request-response communication patterns.

## Learning Objectives
By the end of this lesson, you should be able to:
- Create ROS 2 nodes in Python
- Implement publishers and subscribers for topic communication
- Create and use services for request-response communication
- Understand the differences between topics and services

## Creating a Simple Node
A ROS 2 node is an executable that uses ROS 2 to communicate with other nodes. Let's create a simple Python node:

### Python Node Example
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Hello from simple node!')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publisher and Subscriber
Topics use a publish-subscribe communication model where data is sent asynchronously.

### Publisher Example
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Subscriber Example
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services: Request-Response Communication
Services provide synchronous communication where a client sends a request and waits for a response.

### Service Server Example
```python
#!/usr/bin/env python3

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Client Example
```python
#!/usr/bin/env python3

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' % (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercise
1. Create a new ROS 2 package for this lesson:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python my_robot_tutorials
   cd my_robot_tutorials
   ```

2. Create a publisher node in `my_robot_tutorials/my_robot_tutorials/publisher_member_function.py` with the publisher code above.

3. Create a subscriber node in `my_robot_tutorials/my_robot_tutorials/subscriber_member_function.py` with the subscriber code above.

4. Update the `setup.py` file to include your new nodes as executables.

5. Build and source your workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select my_robot_tutorials
   source install/setup.bash
   ```

6. Run the publisher and subscriber in separate terminals:
   ```bash
   ros2 run my_robot_tutorials publisher_member_function
   ros2 run my_robot_tutorials subscriber_member_function
   ```

## Summary
This lesson provided practical examples of implementing ROS 2 nodes, topics, and services. Understanding these communication patterns is crucial for building distributed robotic systems. The next lesson will cover bridging Python agents to ROS controllers using rclpy.
</ChapterTranslator>