import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class OdomAndWheelTransformPublisher(Node):
    def __init__(self):
        super().__init__('odom_and_wheel_transform_publisher')
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.robot_base_frame_id = "base_link"
        self.odom_frame_id = "odom"
        self.left_wheel_frame_id = "rear_wheel_link_2_1"
        self.right_wheel_frame_id = "rear_wheel_link_1_1"
        
        self.wheel_radius = 0.1  # Adjust based on your actual wheel radius
        self.left_wheel_rotation = 0.0
        self.right_wheel_rotation = 0.0
    
    def odom_callback(self, msg):
        # Calculate the rotation of the wheels based on the distance traveled
        left_wheel_delta = msg.twist.twist.linear.x - msg.twist.twist.angular.z * self.wheel_radius
        right_wheel_delta = msg.twist.twist.linear.x + msg.twist.twist.angular.z * self.wheel_radius
        
        # Update the wheel rotation angles (in radians)
        self.left_wheel_rotation += left_wheel_delta / self.wheel_radius
        self.right_wheel_rotation += right_wheel_delta / self.wheel_radius

        # Broadcast the odom -> base_link transform
        self.broadcast_base_link_transform(msg)
        
        # Broadcast the base_link -> wheels transforms
        self.broadcast_wheel_transform(self.left_wheel_frame_id, 0.264, self.left_wheel_rotation)
        self.broadcast_wheel_transform(self.right_wheel_frame_id, -0.264, self.right_wheel_rotation)

    def broadcast_base_link_transform(self, msg):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.robot_base_frame_id
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        self.tf_broadcaster.sendTransform(t)
    
    def broadcast_wheel_transform(self, child_frame_id, y_offset, wheel_rotation):
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.robot_base_frame_id
        t.child_frame_id = child_frame_id
        
        t.transform.translation.x = -0.1524  # Adjust based on the actual wheel position relative to base_link
        t.transform.translation.y = y_offset  # Offset for left/right wheel
        t.transform.translation.z = 0.1  # Assuming a fixed height
        
        # Set the rotation of the wheel (rotation about the y-axis)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(wheel_rotation / 2.0)
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(wheel_rotation / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

    def listener_callback(self, msg):
        left_wheel_position = msg.pose.pose.position.x
        right_wheel_position = msg.pose.pose.position.y
        self.get_logger().info('Left Wheel Position: %f, Right Wheel Position: %f' % (left_wheel_position, right_wheel_position))

def main(args=None):
    rclpy.init(args=args)
    node = OdomAndWheelTransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import math

# class OdomAndWheelTransformPublisher(Node):
#     def __init__(self):
#         super().__init__('odom_and_wheel_transform_publisher')
        
#         # Subscribe to the odometry topic published by the ESP32
#         self.odom_subscriber = self.create_subscription(
#             Odometry,
#             '/odom',
#             self.odom_callback,
#             10
#         )
        
#         # Create a TransformBroadcaster to send the odom -> base_link and wheel transforms
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         self.robot_base_frame_id = "base_link"
#         self.odom_frame_id = "odom"
#         self.left_wheel_frame_id = "rear_wheel_link_2_1"
#         self.right_wheel_frame_id = "rear_wheel_link_1_1"
    
#     def odom_callback(self, msg):
#         # Broadcast the odom -> base_link transform
#         self.broadcast_base_link_transform(msg)
        
#         # Broadcast the base_link -> wheels transforms
#         self.broadcast_wheel_transform(self.left_wheel_frame_id, 0.264)
#         self.broadcast_wheel_transform(self.right_wheel_frame_id, -0.264)

#     def broadcast_base_link_transform(self, msg):
#         t = TransformStamped()
        
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.odom_frame_id
#         t.child_frame_id = self.robot_base_frame_id
        
#         t.transform.translation.x = msg.pose.pose.position.x
#         t.transform.translation.y = msg.pose.pose.position.y
#         t.transform.translation.z = 0.0
        
#         t.transform.rotation.x = msg.pose.pose.orientation.x
#         t.transform.rotation.y = msg.pose.pose.orientation.y
#         t.transform.rotation.z = msg.pose.pose.orientation.z
#         t.transform.rotation.w = msg.pose.pose.orientation.w
        
#         self.tf_broadcaster.sendTransform(t)
    
#     def broadcast_wheel_transform(self, child_frame_id, y_offset):
#         t = TransformStamped()
        
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.robot_base_frame_id
#         t.child_frame_id = child_frame_id
        
#         # Set the translation for the wheel frame relative to the base_link
#         t.transform.translation.x = -0.1524  # Adjust based on the actual wheel position relative to base_link
#         t.transform.translation.y = y_offset  # Offset for left/right wheel
#         t.transform.translation.z = 0.1  # Assuming a fixed height
        
#         # Assuming no rotation of the wheels relative to base_link in this context
#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = 0.0
#         t.transform.rotation.z = msg.pose.pose.orientation.z
#         t.transform.rotation.w = 1.0
        
#         self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomAndWheelTransformPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import numpy as np

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
        
#         # Wheel separation (distance between wheels)
#         self.wheel_separation = 0.528  # Example value in meters, adjust as necessary
        
#         # Store the last wheel positions
#         self.last_left_wheel_position = 0.0 
#         self.last_right_wheel_position = 0.0
    
#     def odom_callback(self, msg):
#         # Extract robot linear velocity and angular velocity from the odom message
#         linear_velocity = msg.twist.twist.linear.x
#         angular_velocity = msg.twist.twist.angular.z
        
#         # Compute individual wheel velocities (assuming differential drive)
#         left_wheel_velocity = linear_velocity - (angular_velocity * self.wheel_separation / 2)
#         right_wheel_velocity = linear_velocity + (angular_velocity * self.wheel_separation / 2)
        
#         # Assuming the odom message is published at a fixed rate, compute the time delta
#         current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
#                        self.get_clock().now().seconds_nanoseconds()[1] / 1e9
#         time_delta = current_time - msg.header.stamp.sec - msg.header.stamp.nanosec / 1e9
        
#         # Calculate the change in wheel positions (delta)
#         delta_left_wheel = left_wheel_velocity * time_delta
#         delta_right_wheel = right_wheel_velocity * time_delta
        
#         # Update the last known positions
#         self.last_left_wheel_position += delta_left_wheel
#         self.last_right_wheel_position += delta_right_wheel
        
#         # Publish the updated wheel transforms
#         self.broadcast_wheel_transform(self.left_wheel_frame_id, self.last_left_wheel_position)
#         self.broadcast_wheel_transform(self.right_wheel_frame_id, self.last_right_wheel_position)
    
#     def broadcast_wheel_transform(self, child_frame_id, wheel_position):
#         t = TransformStamped()
        
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.robot_base_frame_id
#         t.child_frame_id = child_frame_id
        
#         # Set translation based on wheel positions
#         t.transform.translation.x = wheel_position
        
#         if child_frame_id == self.left_wheel_frame_id:
#             t.transform.translation.y = self.wheel_separation / 2
#         else:
#             t.transform.translation.y = -self.wheel_separation / 2
#         t.transform.translation.z = 0.1
        
#         # Assume no rotation for simplicity (can be updated with rotation logic)
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


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import numpy as np

# class WheelTransformPublisher(Node):
#     def __init__(self):
#         super().__init__('wheel_transform_publisher')
        
#         self.joint_state_subscriber = self.create_subscription(
#             JointState,
#             '/joint_states',
#             self.joint_state_callback,
#             10
#         )
        
#         self.tf_broadcaster = TransformBroadcaster(self)
        
#         self.robot_base_frame_id = "base_link"
#         self.left_wheel_frame_id = "rear_wheel_link_2_1"
#         self.right_wheel_frame_id = "rear_wheel_link_1_1"
        
#         # Store the last wheel positions
#         self.last_left_wheel_position = 0.0
#         self.last_right_wheel_position = 0.0
    
#     def joint_state_callback(self, msg):
#         # Extract the wheel positions from the joint state message
#         try:
#             left_wheel_index = msg.name.index("base_to_left_wheel")
#             right_wheel_index = msg.name.index("base_to_right_wheel")
            
#             left_wheel_position = msg.position[left_wheel_index]
#             right_wheel_position = msg.position[right_wheel_index]
            
#             # Calculate the change in wheel positions (delta)
#             delta_left_wheel = left_wheel_position - self.last_left_wheel_position
#             delta_right_wheel = right_wheel_position - self.last_right_wheel_position
            
#             # Update the last known positions
#             self.last_left_wheel_position = left_wheel_position
#             self.last_right_wheel_position = right_wheel_position
            
#             # Publish the updated wheel transforms
#             self.broadcast_wheel_transform(self.left_wheel_frame_id, delta_left_wheel)
#             self.broadcast_wheel_transform(self.right_wheel_frame_id, delta_right_wheel)
        
#         except ValueError:
#             self.get_logger().error("Joint names for wheels not found in JointState message")
    
#     def broadcast_wheel_transform(self, child_frame_id, wheel_position_delta):
#         t = TransformStamped()
        
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.robot_base_frame_id
#         t.child_frame_id = child_frame_id
        
#         # Assuming the wheels only rotate around the y-axis
#         t.transform.translation.x = -0.1524
#         if child_frame_id == self.left_wheel_frame_id:
#             t.transform.translation.y = 0.264
#         else:
#             t.transform.translation.y = -0.264
#         t.transform.translation.z = 0.1
        
#         # Rotation about the y-axis (pitch)
#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = np.sin(wheel_position_delta / 2)
#         t.transform.rotation.z = 0.0
#         t.transform.rotation.w = np.cos(wheel_position_delta / 2)
        
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
















# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# import numpy as np

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
        
#         # Store the last wheel positions
#         self.last_left_wheel_position = 0.0
#         self.last_right_wheel_position = 0.0
    
#     def odom_callback(self, msg):
#         # Assuming the position information for the wheels is derived from the odometry data
#         # Modify this part to extract the correct wheel position data from the odometry message
#         # For demonstration, we will assume the data is stored in `msg.pose.pose.position.x` and `msg.pose.pose.position.y`
        
#         left_wheel_position = msg.pose.pose.position.x  # Replace with actual left wheel position extraction
#         right_wheel_position = msg.pose.pose.position.y  # Replace with actual right wheel position extraction
        
#         # Calculate the change in wheel positions (delta)
#         delta_left_wheel = left_wheel_position - self.last_left_wheel_position
#         delta_right_wheel = right_wheel_position - self.last_right_wheel_position
        
#         # Update the last known positions
#         self.last_left_wheel_position = left_wheel_position
#         self.last_right_wheel_position = right_wheel_position
        
#         # Publish the updated wheel transforms
#         self.broadcast_wheel_transform(self.left_wheel_frame_id, delta_left_wheel)
#         self.broadcast_wheel_transform(self.right_wheel_frame_id, delta_right_wheel)
    
#     def broadcast_wheel_transform(self, child_frame_id, wheel_position_delta):
#         t = TransformStamped()
        
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self.robot_base_frame_id
#         t.child_frame_id = child_frame_id
        
#         # Assuming the wheels only rotate around the y-axis
#         t.transform.translation.x = -0.1524= left_wheel_position - self.last_left_wheel_position
#         delta_right_wheel 
#         if child_frame_id == self.left_wheel_frame_id:
#             t.transform.translation.y = 0.264
#         else:
#             t.transform.translation.y = -0.264
#         t.transform.translation.z = 0.1
        
#         # Rotation about the y-axis (pitch)
#         t.transform.rotation.x = 0.0
#         t.transform.rotation.y = np.sin(wheel_position_delta / 2)
#         t.transform.rotation.z = 0.0
#         t.transform.rotation.w = np.cos(wheel_position_delta / 2)
        
#         self.tf_broadcaster.sendTransform(t)

# def main(args=None):
#     rclpy.init(args=args)
#     node = WheelTransformPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

