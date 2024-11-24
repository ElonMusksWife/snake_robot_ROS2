#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from std_msgs.msg import Int32

import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')

        # Publisher for setting positions
        self.publisher = self.create_publisher(SetPosition, '/set_position', 10)
        self.get_position_client = self.create_client(GetPosition, '/get_position')

        # ADD publishers for target and current positions. 
        self.target_position_pub = self.create_publisher(Int32, '/motor1/target_position', 10)
        # self.current_position_pub = self.create_publisher(Int32, '/motor5/current_position', 10)

        
        # self.timer = self.create_timer(0.1, self.publish_current_position)
        # self.joint_id = 5

    # Publish target and current position
    # def publish_current_position(self):
    #     """
    #     Continuously query and publish the current position of motor 1.
    #     """
    #     joint_id = 5
    #     request = GetPosition.Request()
    #     request.id = joint_id

    #     # Wait for the service to be available
    #     if self.get_position_client.wait_for_service(timeout_sec=0.5):
    #         future = self.get_position_client.call_async(request)
    #         rclpy.spin_until_future_complete(self, future)

    #         if future.result() is not None:
    #             current_position = future.result().position
    #             current_msg = Int32()
    #             current_msg.data = current_position
    #             self.current_position_pub.publish(current_msg)
    #             self.get_logger().info(f'Motor {joint_id} Current Position: {current_position}')
    #         else:
    #             self.get_logger().error('Failed to get current position')
    #     else:
    #         self.get_logger().warn('Waiting for /get_position service to become available')


    def publish_target(self,joint_id, target_position):
        target_msg = Int32()
        target_msg.data = int(target_position)  # Ensure it's an integer
        self.target_position_pub.publish(target_msg)
        self.get_logger().info(
                f"Motor {joint_id}: Target={target_position}")



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
            time.sleep(0.1)  # Add a 100ms
    
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
        
        # Main code go here
        joint = 3
        deg = 30
        for k in range(50):

            target_position = int(joint_controller.angle_to_position(-30))
            joint_controller.publish_target(joint, target_position)
            
            joint_angles = [-deg,deg, -deg, deg,-deg] 
            joint_controller.set_joint_positions(joint_angles)
            # time.sleep(0.2)  # Add a 1000ms
            joint_angles = [deg,-deg, deg, -deg, deg] 
            joint_controller.set_joint_positions(joint_angles)

            # target_position = int(joint_controller.angle_to_position(-30))
            # joint_controller.publish_target(joint, target_position)
            

            

            # target_position = int(joint_controller.angle_to_position(0))
            # joint_controller.publish_target(joint, target_position)

            # joint_angles = [deg,deg,30,deg,deg] 
            # joint_controller.set_joint_positions(joint_angles)

            # target_position = int(joint_controller.angle_to_position(0))
            # joint_controller.publish_target(joint, target_position)
        
        
        
        
        
        # target_position = int(joint_controller.angle_to_position(0))
        # joint_controller.publish_target(joint, target_position)

        # time.sleep(1)
        # target_position = int(joint_controller.angle_to_position(0))
        # joint_controller.publish_target(joint, target_position)
        # target_position = int(joint_controller.angle_to_position(-90))
        # joint_controller.publish_target(joint, target_position)
        
        # current = 0
        # # for i in range(0,-91,-1):
        # for i in range(1, 92):
        #     target_position = int(joint_controller.angle_to_position(-90))
        #     joint_controller.publish_target(joint, target_position)
        #     joint_angles = [deg,deg,deg,deg,i]  
        #     joint_controller.set_joint_positions(joint_angles)
            
        

        
        # rclpy.spin(joint_controller)
    except KeyboardInterrupt:
        print('Shutting down...')
    finally:
        joint_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
