#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition

import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publisher for setting positions
        self.publisher = self.create_publisher(SetPosition, '/set_position', 10)
        self.get_position_client = self.create_client(GetPosition, '/get_position')

    @staticmethod
    def angle_to_position(angle_degrees):
        """
        Convert angle in degrees to Dynamixel position.
        Formula: position = 2048 - (degrees * 11.38)
        """
        pos = 2048
        if(angle_degrees <= 90 and angle_degrees >= -90):
            pos = int(2048 - angle_degrees * 11.3888)
            
        return pos

    @staticmethod
    def position_to_angle(position):
        """
        Convert Dynamixel position back to angle in degrees.
        """
        return (2048 - position) / 11.38

    def set_joint_positions(self, joint_angles):
        """
        Publish positions for all joints.
        :param joint_angles: List of angles in degrees for joints [joint1, joint2, ..., joint5]
        """
        for joint_id, angle in enumerate(joint_angles, start=1):
            position = self.angle_to_position(angle)
            msg = SetPosition()
            msg.id = joint_id
            msg.position = position
            self.publisher.publish(msg)
            self.get_logger().info(f'Set joint {joint_id} to angle {angle}° (position: {position})')
            time.sleep(0.2)  # Add a 100ms
    
    def enable_torque(self, joint_ids):
        for joint_id in joint_ids:
            msg = SetPosition()
            msg.id = joint_id
            # msg.position = 0  # Enabling torque doesn't need a position change
            msg.position = 2048 # Enabling torque doesn't need a position change
            self.publisher.publish(msg)
            self.get_logger().info(f'Enabled torque for joint {joint_id}')


    def get_joint_position(self, joint_id):
        """
        Request the current position of a joint via the service.
        :param joint_id: ID of the joint
        :return: Current position as an integer or None if the service call fails
        """
        request = GetPosition.Request()
        request.id = joint_id

        # Wait for the service to be available
        while not self.get_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /get_position service...')
        
        # Call the service
        future = self.get_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            position = future.result().position
            angle = self.position_to_angle(position)
            self.get_logger().info(f'Joint {joint_id} is at position {position} (angle: {angle:.2f}°)')
            return position
        else:
            self.get_logger().error('Failed to call /get_position service')
            return None


def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()

    try:
        joint_ids = [1, 2, 3, 4, 5]
        joint_controller.enable_torque(joint_ids)
        deg = 0
        # Example: Set positions for joints 1 to 5
        # joint_angles = [50,-50,50,-50,50]  # Angles in degrees
        joint_angles = [deg,deg,deg,deg,deg]  # Angles in degrees
        # joint_angles = [0.1,0.1,0.1,0.1,0.1]  # Angles in degrees
        joint_controller.set_joint_positions(joint_angles)

        # Example: Get the position of joint 2
        joint_controller.get_joint_position(2)

    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        joint_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
