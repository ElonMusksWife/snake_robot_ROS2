import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from dynamixel_sdk_custom_interfaces.srv import GetPosition


class MotorPositionServicePublisher(Node):
    def __init__(self):
        super().__init__('motor_position_service_publisher')

        # Create a publisher for the /motor5/current_position topic
        self.publisher_ = self.create_publisher(Int32, '/motor5/current_position', 10)

        # Create a client for the /get_position service
        self.client = self.create_client(GetPosition, '/get_position')

        # Wait for the service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /get_position service...')

        # Timer to call the service periodically (e.g., every second)
        self.timer = self.create_timer(0.25, self.timer_callback)

        self.get_logger().info('Motor Position Service Publisher Node has started!')

    def timer_callback(self):
        # Prepare a request for motor ID 5
        request = GetPosition.Request()
        request.id = 5

        # Call the service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            # Since the response only contains 'position', log it
            self.get_logger().info(f'Received position: {response.position}')

            # Publish the position to the /motor5/current_position topic
            msg = Int32()
            msg.data = response.position
            if msg.data < 5000:
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published position {response.position} to /motor5/current_position topic')
        except Exception as e:
            self.get_logger().error(f'Failed to call /get_position service: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorPositionServicePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
