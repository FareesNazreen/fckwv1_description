# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import math

# class WheelTransformPublisher(Node):
#     def __init__(self):
#         super().__init__('wheel_transform_publisher')
        
#         # Subscription to the /odom topic
#         self.odom_subscriber = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10
#         )
        
#         # TF Broadcaster
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         self.left_wheel_frame_id = "rear_wheel_link_1_1"
#         self.right_wheel_frame_id = "rear_wheel_link_2_1"
#         self.robot_base_frame_id = "base_link"
        
#         self.wheel_separation = 0.5  # Distance between the wheels
        
#     def odom_callback(self, msg):
#         # Extract position and orientation
#         x = msg.pose.pose.position.x
#         y = msg.pose.pose.position.y
#         z = msg.pose.pose.position.z
        
#         orientation = msg.pose.pose.orientation
#         yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
#                          1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
        
#         # Calculate positions of left and right wheels
#         left_wheel_x = x - (self.wheel_separation / 2.0) * math.sin(yaw)
#         left_wheel_y = y + (self.wheel_separation / 2.0) * math.cos(yaw)
#         right_wheel_x = x + (self.wheel_separation / 2.0) * math.sin(yaw)
#         right_wheel_y = y - (self.wheel_separation / 2.0) * math.cos(yaw)
        
#         # Publish transforms
#         self.publish_transform(left_wheel_x, left_wheel_y, z, orientation, self.left_wheel_frame_id)
#         self.publish_transform(right_wheel_x, right_wheel_y, z, orientation, self.right_wheel_frame_id)
    
#     def publish_transform(self, x, y, z, orientation, child_frame_id):
#         t = TransformStamped()
        
#         # Set time and frame information
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.robot_base_frame_id
#         t.child_frame_id = child_frame_id
        
#         # Set translation and rotation
#         t.transform.translation.x = x
#         t.transform.translation.y = y
#         t.transform.translation.z = z
#         t.transform.rotation = orientation
        
#         # Send the transform
#         self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = WheelTransformPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

# class WheelTransformPublisher(Node):
#     def __init__(self):
#         super().__init__('wheel_transform_publisher')
        
#         self.odom_subscriber = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10
#         )
        
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         self.robot_base_frame_id = "base_link"
#         self.left_wheel_frame_id = "rear_wheel_link_2_1"
#         self.right_wheel_frame_id = "rear_wheel_link_1_1"
#         self.odom_frame_id = "odom"
        
#     def odom_callback(self, msg):
#         # Publish odom to base_link transform
#         self.broadcast_transform(msg.pose.pose, self.odom_frame_id, self.robot_base_frame_id)
        
#         # Publish base_link to left_wheel and right_wheel transforms
#         self.broadcast_wheel_transform(self.left_wheel_frame_id, 0.264)
#         self.broadcast_wheel_transform(self.right_wheel_frame_id, -0.264)
    
#     def broadcast_transform(self, pose, parent_frame_id, child_frame_id):
#         t = TransformStamped()
        
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = parent_frame_id
#         t.child_frame_id = child_frame_id
        
#         t.transform.translation.x = pose.position.x
#         t.transform.translation.y = pose.position.y
#         t.transform.translation.z = pose.position.z
#         t.transform.rotation = pose.orientation
        
#         self.tf_broadcaster.sendTransform(t)
    
#     def broadcast_wheel_transform(self, child_frame_id, y_offset):
#         t = TransformStamped()
        
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.robot_base_frame_id
#         t.child_frame_id = child_frame_id
        
#         # Use positions from URDF
#         t.transform.translation.x = -0.1524  # x position for both wheels
#         t.transform.translation.y = y_offset  # y position: positive for left, negative for right
#         t.transform.translation.z = 0.1  # z position from URDF
        
#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = 0.0
#         t.transform.rotation.z = 0.0
#         t.transform.rotation.w = 1.0
        
#         self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = WheelTransformPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class WheelTransformPublisher(Node):
    def __init__(self):
        super().__init__('wheel_transform_publisher')
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.robot_base_frame_id = "base_link"
        self.left_wheel_frame_id = "rear_wheel_link_2_1"
        self.right_wheel_frame_id = "rear_wheel_link_1_1"
        
        # Store the last wheel positions
        self.last_left_wheel_position = 0.0
        self.last_right_wheel_position = 0.0
    
    def joint_state_callback(self, msg):
        # Extract the wheel positions from the joint state message
        try:
            left_wheel_index = msg.name.index("base_to_left_wheel")
            right_wheel_index = msg.name.index("base_to_right_wheel")
            
            left_wheel_position = msg.position[left_wheel_index]
            right_wheel_position = msg.position[right_wheel_index]
            
            # Calculate the change in wheel positions (delta)
            delta_left_wheel = left_wheel_position - self.last_left_wheel_position
            delta_right_wheel = right_wheel_position - self.last_right_wheel_position
            
            # Update the last known positions
            self.last_left_wheel_position = left_wheel_position
            self.last_right_wheel_position = right_wheel_position
            
            # Publish the updated wheel transforms
            self.broadcast_wheel_transform(self.left_wheel_frame_id, delta_left_wheel)
            self.broadcast_wheel_transform(self.right_wheel_frame_id, delta_right_wheel)
        
        except ValueError:
            self.get_logger().error("Joint names for wheels not found in JointState message")
    
    def broadcast_wheel_transform(self, child_frame_id, wheel_position_delta):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base_frame_id
        t.child_frame_id = child_frame_id
        
        # Assuming the wheels only rotate around the y-axis
        t.transform.translation.x = -0.1524
        if child_frame_id == self.left_wheel_frame_id:
            t.transform.translation.y = 0.264
        else:
            t.transform.translation.y = -0.264
        t.transform.translation.z = 0.1
        
        # Rotation about the y-axis (pitch)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = np.sin(wheel_position_delta / 2)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = np.cos(wheel_position_delta / 2)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = WheelTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



